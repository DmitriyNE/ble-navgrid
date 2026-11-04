//! Backing storage for a replica's tables: fixed capacity on devices, growable on the host.
//!
//! Devices keep records and outbox entries postcard-encoded end to end in an [`Arena`], at about
//! their wire size, instead of as structs sized by the largest record (DEF-29); the host keeps
//! structs. So a table hands its entries out by value, and the replica searches it through
//! [`Table::with`].

use core::cmp::Ordering;
use core::marker::PhantomData;

use serde::Serialize;
use serde::de::DeserializeOwned;

use crate::replica::{Pending, Route, Stored};

/// A table of `T`, which the replica keeps sorted. Entries come out by value, as a table may
/// hold them encoded.
pub trait Table<T> {
    fn len(&self) -> usize;

    fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Runs `f` on the entry at `index`.
    fn with<R>(&self, index: usize, f: impl FnOnce(&T) -> R) -> R;

    /// The entry at `index`. (Not `get`: on a `heapless::Vec` that would shadow the slice's.)
    fn entry(&self, index: usize) -> T
    where
        T: Clone,
    {
        self.with(index, T::clone)
    }

    /// Replaces the entry at `index`, or hands the value back when it no longer fits.
    fn set(&mut self, index: usize, value: T) -> Result<(), T>;

    /// Inserts at `index`, or hands the value back when the table is full.
    fn insert(&mut self, index: usize, value: T) -> Result<(), T>;

    fn remove(&mut self, index: usize);

    fn retain(&mut self, keep: impl FnMut(&T) -> bool);

    fn push(&mut self, value: T) -> Result<(), T> {
        self.insert(self.len(), value)
    }

    /// The entries in table order. (Not `iter`, for the same reason.)
    fn entries(&self) -> impl ExactSizeIterator<Item = T> + '_
    where
        T: Clone,
    {
        (0..self.len()).map(move |i| self.entry(i))
    }

    /// As the slice method, over the entries in table order.
    fn binary_search_by(&self, mut f: impl FnMut(&T) -> Ordering) -> Result<usize, usize> {
        let (mut lo, mut hi) = (0, self.len());
        while lo < hi {
            let mid = lo + (hi - lo) / 2;
            match self.with(mid, &mut f) {
                Ordering::Less => lo = mid + 1,
                Ordering::Greater => hi = mid,
                Ordering::Equal => return Ok(mid),
            }
        }
        Err(lo)
    }

    /// As the slice method: the index of the first entry for which `pred` is false, the entries
    /// being partitioned.
    fn partition_point(&self, mut pred: impl FnMut(&T) -> bool) -> usize {
        self.binary_search_by(|t| if pred(t) { Ordering::Less } else { Ordering::Greater })
            .unwrap_or_else(|i| i)
    }
}

impl<T, const N: usize> Table<T> for heapless::Vec<T, N> {
    fn len(&self) -> usize {
        self.as_slice().len()
    }

    fn with<R>(&self, index: usize, f: impl FnOnce(&T) -> R) -> R {
        f(&self.as_slice()[index])
    }

    fn set(&mut self, index: usize, value: T) -> Result<(), T> {
        self.as_mut_slice()[index] = value;
        Ok(())
    }

    fn insert(&mut self, index: usize, value: T) -> Result<(), T> {
        heapless::Vec::insert(self, index, value)
    }

    fn remove(&mut self, index: usize) {
        heapless::Vec::remove(self, index);
    }

    fn retain(&mut self, keep: impl FnMut(&T) -> bool) {
        heapless::Vec::retain(self, keep)
    }
}

#[cfg(feature = "alloc")]
impl<T> Table<T> for alloc::vec::Vec<T> {
    fn len(&self) -> usize {
        self.as_slice().len()
    }

    fn with<R>(&self, index: usize, f: impl FnOnce(&T) -> R) -> R {
        f(&self.as_slice()[index])
    }

    fn set(&mut self, index: usize, value: T) -> Result<(), T> {
        self.as_mut_slice()[index] = value;
        Ok(())
    }

    fn insert(&mut self, index: usize, value: T) -> Result<(), T> {
        alloc::vec::Vec::insert(self, index, value);
        Ok(())
    }

    fn remove(&mut self, index: usize) {
        alloc::vec::Vec::remove(self, index);
    }

    fn retain(&mut self, keep: impl FnMut(&T) -> bool) {
        alloc::vec::Vec::retain(self, keep)
    }
}

/// A table lent to a replica: a device keeps its arenas in statics and hands the replica
/// references, so the replica itself stays small enough to build on the stack.
pub struct Lent<A: 'static>(pub &'static mut A);

impl<T, A: Table<T>> Table<T> for Lent<A> {
    fn len(&self) -> usize {
        A::len(self.0)
    }

    fn with<R>(&self, index: usize, f: impl FnOnce(&T) -> R) -> R {
        A::with(self.0, index, f)
    }

    fn set(&mut self, index: usize, value: T) -> Result<(), T> {
        A::set(self.0, index, value)
    }

    fn insert(&mut self, index: usize, value: T) -> Result<(), T> {
        A::insert(self.0, index, value)
    }

    fn remove(&mut self, index: usize) {
        A::remove(self.0, index)
    }

    fn retain(&mut self, keep: impl FnMut(&T) -> bool) {
        A::retain(self.0, keep)
    }
}

/// The longest encoding of an entry: a record at its wire maximum, plus the table's own fields.
const ENTRY_MAX: usize = navgrid_proto::MESH_RECORD_LEN + 32;

/// Entries postcard-encoded end to end in `BYTES` (under 64 KiB), with an index of where each
/// lies, in table order; `ENTRIES` of them at most. A record takes about its wire size, 40–60 B,
/// against the 136 B of its struct (DEF-29). Removing an entry moves the ones after it down.
pub struct Arena<T, const BYTES: usize, const ENTRIES: usize> {
    bytes: [u8; BYTES],
    used: usize,
    /// Offset and length of each entry, in table order.
    index: heapless::Vec<(u16, u16), ENTRIES>,
    entries: PhantomData<T>,
}

impl<T, const BYTES: usize, const ENTRIES: usize> Arena<T, BYTES, ENTRIES> {
    /// An empty arena; `const`, so a `static` one lives in `.bss` and costs no stack.
    pub const fn new() -> Self {
        const { assert!(BYTES <= u16::MAX as usize + 1, "an arena's offsets are 16-bit") };
        Self {
            bytes: [0; BYTES],
            used: 0,
            index: heapless::Vec::new(),
            entries: PhantomData,
        }
    }

    /// Bytes of entries held, of `BYTES`.
    pub fn used(&self) -> usize {
        self.used
    }

    fn encoded(&self, index: usize) -> &[u8] {
        let (at, len) = self.index[index];
        &self.bytes[usize::from(at)..usize::from(at) + usize::from(len)]
    }

    fn take_out(&mut self, index: usize) {
        let (at, len) = self.index.remove(index);
        let (at, len) = (usize::from(at), usize::from(len));
        self.bytes.copy_within(at + len..self.used, at);
        self.used -= len;
        for (offset, _) in self.index.iter_mut() {
            if usize::from(*offset) > at {
                *offset -= len as u16;
            }
        }
    }

    fn put_in(&mut self, index: usize, encoded: &[u8]) {
        let at = self.used;
        self.bytes[at..at + encoded.len()].copy_from_slice(encoded);
        self.used += encoded.len();
        // Room was checked by the caller.
        let _ = self.index.insert(index, (at as u16, encoded.len() as u16));
    }
}

impl<T, const BYTES: usize, const ENTRIES: usize> Default for Arena<T, BYTES, ENTRIES> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Serialize + DeserializeOwned, const BYTES: usize, const ENTRIES: usize> Table<T> for Arena<T, BYTES, ENTRIES> {
    fn len(&self) -> usize {
        self.index.len()
    }

    fn with<R>(&self, index: usize, f: impl FnOnce(&T) -> R) -> R {
        let entry: T = postcard::from_bytes(self.encoded(index)).expect("an arena entry decodes as it was encoded");
        f(&entry)
    }

    fn set(&mut self, index: usize, value: T) -> Result<(), T> {
        let mut buf = [0u8; ENTRY_MAX];
        let n = match postcard::to_slice(&value, &mut buf) {
            Ok(encoded) => encoded.len(),
            Err(_) => return Err(value),
        };
        let old = usize::from(self.index[index].1);
        if self.used - old + n > BYTES {
            return Err(value);
        }
        self.take_out(index);
        self.put_in(index, &buf[..n]);
        Ok(())
    }

    fn insert(&mut self, index: usize, value: T) -> Result<(), T> {
        let mut buf = [0u8; ENTRY_MAX];
        let n = match postcard::to_slice(&value, &mut buf) {
            Ok(encoded) => encoded.len(),
            Err(_) => return Err(value),
        };
        if self.index.is_full() || self.used + n > BYTES {
            return Err(value);
        }
        self.put_in(index, &buf[..n]);
        Ok(())
    }

    fn remove(&mut self, index: usize) {
        self.take_out(index);
    }

    fn retain(&mut self, mut keep: impl FnMut(&T) -> bool) {
        for i in (0..self.index.len()).rev() {
            if !self.with(i, &mut keep) {
                self.take_out(i);
            }
        }
    }
}

/// The tables of a [`Replica`](crate::Replica).
pub trait Storage {
    type Records: Table<Stored>;
    type Outbox: Table<Pending>;
    type Routes: Table<Route>;
}

/// Fixed-capacity tables of structs, for tests of the bounds.
pub struct Fixed<const RECORDS: usize, const OUTBOX: usize, const ROUTES: usize>;

impl<const RECORDS: usize, const OUTBOX: usize, const ROUTES: usize> Storage for Fixed<RECORDS, OUTBOX, ROUTES> {
    type Records = heapless::Vec<Stored, RECORDS>;
    type Outbox = heapless::Vec<Pending, OUTBOX>;
    type Routes = heapless::Vec<Route, ROUTES>;
}

/// A device's tables: records and the outbox in static arenas the replica borrows
/// ([`Replica::with_tables`](crate::Replica::with_tables)), routes as structs.
pub struct Device<
    const RECORD_BYTES: usize,
    const RECORDS: usize,
    const OUTBOX_BYTES: usize,
    const OUTBOX: usize,
    const ROUTES: usize,
>;

impl<
    const RECORD_BYTES: usize,
    const RECORDS: usize,
    const OUTBOX_BYTES: usize,
    const OUTBOX: usize,
    const ROUTES: usize,
> Storage for Device<RECORD_BYTES, RECORDS, OUTBOX_BYTES, OUTBOX, ROUTES>
{
    type Records = Lent<Arena<Stored, RECORD_BYTES, RECORDS>>;
    type Outbox = Lent<Arena<Pending, OUTBOX_BYTES, OUTBOX>>;
    type Routes = Lent<heapless::Vec<Route, ROUTES>>;
}

/// Growable tables, for the host: sinks behind a host, and the simulator, which caps them with
/// [`Limits`](crate::Limits).
#[cfg(feature = "alloc")]
pub struct Heap;

#[cfg(feature = "alloc")]
impl Storage for Heap {
    type Records = alloc::vec::Vec<Stored>;
    type Outbox = alloc::vec::Vec<Pending>;
    type Routes = alloc::vec::Vec<Route>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_arena_keeps_table_order_and_compacts() {
        let mut a: Arena<u32, 64, 8> = Arena::new();
        assert!(a.insert(0, 10).is_ok());
        assert!(a.insert(0, 5).is_ok());
        assert!(a.insert(2, 300_000).is_ok());
        assert_eq!(a.entries().collect::<alloc::vec::Vec<_>>(), [5, 10, 300_000]);
        let used = a.used();
        a.remove(0);
        assert_eq!(a.entries().collect::<alloc::vec::Vec<_>>(), [10, 300_000]);
        assert!(a.used() < used);
        assert!(a.set(1, 7).is_ok());
        assert_eq!(a.entries().collect::<alloc::vec::Vec<_>>(), [10, 7]);
        a.retain(|&v| v != 10);
        assert_eq!(a.entries().collect::<alloc::vec::Vec<_>>(), [7]);
        assert_eq!(a.binary_search_by(|v| v.cmp(&7)), Ok(0));
        assert_eq!(a.binary_search_by(|v| v.cmp(&9)), Err(1));
    }

    #[test]
    fn an_arena_refuses_what_does_not_fit() {
        let mut a: Arena<u64, 8, 4> = Arena::new();
        // Each u64 near the top encodes as 10 bytes.
        assert_eq!(a.insert(0, u64::MAX), Err(u64::MAX));
        assert!(a.insert(0, 1).is_ok());
        assert!(a.insert(1, 2).is_ok());
        assert_eq!(a.set(0, u64::MAX), Err(u64::MAX), "a longer encoding that won't fit");
        assert_eq!(a.entry(0), 1, "and the entry stays");
        let mut b: Arena<u8, 64, 2> = Arena::new();
        assert!(b.insert(0, 1).is_ok() && b.insert(1, 2).is_ok());
        assert_eq!(b.insert(2, 3), Err(3), "the index is full");
    }
}
