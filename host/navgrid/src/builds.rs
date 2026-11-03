//! The builds kept in the fleet directory (`Fleet::keep_build`), for what devices running them
//! report: their `defmt` log frames and the addresses in their crash records (architecture §13).

use std::collections::HashMap;
use std::path::PathBuf;

use navgrid_proto::{Board, Firmware};
use object::{Object, ObjectSymbol, SymbolKind};

pub struct Builds {
    dir: PathBuf,
    /// Each build looked up so far, `None` for one that isn't kept or doesn't parse.
    loaded: HashMap<(Firmware, Board), Option<Build>>,
}

struct Build {
    table: defmt_decoder::Table,
    locations: defmt_decoder::Locations,
    /// Functions: start address, size and demangled name.
    functions: Vec<(u64, u64, String)>,
}

impl Builds {
    pub fn new(dir: PathBuf) -> Self {
        Self {
            dir,
            loaded: HashMap::new(),
        }
    }

    fn get(&mut self, firmware: &Firmware, board: Board) -> Option<&Build> {
        let dir = &self.dir;
        self.loaded
            .entry((*firmware, board))
            .or_insert_with(|| load(&dir.join(crate::fleet::build_file(&firmware.to_string(), board))))
            .as_ref()
    }

    /// A log frame as `0.654321 WARN distance 748: too few channels (0) (cs.rs:345)`, or `None`
    /// without the build.
    pub fn log_line(&mut self, firmware: &Firmware, board: Board, frame: &[u8]) -> Option<String> {
        let build = self.get(firmware, board)?;
        let (decoded, _) = build.table.decode(frame).ok()?;
        let level = decoded
            .level()
            .map_or("PRINT".to_owned(), |l| l.as_str().to_uppercase());
        let at = build.locations.get(&decoded.index()).map_or(String::new(), |l| {
            let file = l
                .file
                .file_name()
                .map_or(l.file.display().to_string(), |f| f.to_string_lossy().into());
            format!(" ({file}:{})", l.line)
        });
        let timestamp = decoded.display_timestamp().map_or(String::new(), |t| format!("{t} "));
        Some(format!("{timestamp}{level} {}{at}", decoded.display_message()))
    }

    /// The function holding `pc` in that build.
    pub fn function(&mut self, firmware: &Firmware, board: Board, pc: u32) -> Option<String> {
        let pc = u64::from(pc);
        self.get(firmware, board)?
            .functions
            .iter()
            .find(|(start, size, _)| (*start..start + size).contains(&pc))
            .map(|(_, _, name)| name.clone())
    }
}

fn load(path: &std::path::Path) -> Option<Build> {
    let elf = std::fs::read(path).ok()?;
    let table = defmt_decoder::Table::parse(&elf).ok()??;
    let locations = table.get_locations(&elf).unwrap_or_default();
    let file = object::File::parse(&*elf).ok()?;
    let functions = file
        .symbols()
        .filter(|s| s.kind() == SymbolKind::Text && s.size() > 0)
        .filter_map(|s| {
            let name = s.name().ok()?;
            // Thumb functions have bit 0 set in their address.
            Some((
                s.address() & !1,
                s.size(),
                format!("{:#}", rustc_demangle::demangle(name)),
            ))
        })
        .collect();
    Some(Build {
        table,
        locations,
        functions,
    })
}
