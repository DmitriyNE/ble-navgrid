//! Backing storage for a replica's tables: fixed capacity on devices, growable on the host.

use core::ops::{Deref, DerefMut};

use crate::replica::{Pending, Route, Stored};

/// A table of `T`, which the replica keeps sorted.
pub trait Table<T>: Default + Deref<Target = [T]> + DerefMut {
    /// Inserts at `index`, or hands the value back when the table is full.
    fn insert(&mut self, index: usize, value: T) -> Result<(), T>;
    fn remove(&mut self, index: usize) -> T;
    fn retain(&mut self, keep: impl FnMut(&T) -> bool);

    fn push(&mut self, value: T) -> Result<(), T> {
        self.insert(self.len(), value)
    }
}

impl<T, const N: usize> Table<T> for heapless::Vec<T, N> {
    fn insert(&mut self, index: usize, value: T) -> Result<(), T> {
        heapless::Vec::insert(self, index, value)
    }

    fn remove(&mut self, index: usize) -> T {
        heapless::Vec::remove(self, index)
    }

    fn retain(&mut self, keep: impl FnMut(&T) -> bool) {
        heapless::Vec::retain(self, keep)
    }
}

#[cfg(feature = "alloc")]
impl<T> Table<T> for alloc::vec::Vec<T> {
    fn insert(&mut self, index: usize, value: T) -> Result<(), T> {
        alloc::vec::Vec::insert(self, index, value);
        Ok(())
    }

    fn remove(&mut self, index: usize) -> T {
        alloc::vec::Vec::remove(self, index)
    }

    fn retain(&mut self, keep: impl FnMut(&T) -> bool) {
        alloc::vec::Vec::retain(self, keep)
    }
}

/// The tables of a [`Replica`](crate::Replica).
pub trait Storage {
    type Records: Table<Stored>;
    type Outbox: Table<Pending>;
    type Routes: Table<Route>;
}

/// Fixed-capacity tables, for devices.
pub struct Fixed<const RECORDS: usize, const OUTBOX: usize, const ROUTES: usize>;

impl<const RECORDS: usize, const OUTBOX: usize, const ROUTES: usize> Storage for Fixed<RECORDS, OUTBOX, ROUTES> {
    type Records = heapless::Vec<Stored, RECORDS>;
    type Outbox = heapless::Vec<Pending, OUTBOX>;
    type Routes = heapless::Vec<Route, ROUTES>;
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
