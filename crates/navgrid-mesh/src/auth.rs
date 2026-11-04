//! Authenticating desired state (DEF-27, NFR-7). The controller's key is derived from the fleet
//! key, so every sink and node can check a command, and the host, which holds the fleet key, can
//! sign one. A [`Desired`] record's MAC covers its origin, version, target and command: a relay
//! can carry it but not change or forge it.

use aes::Aes128;
use cmac::{Cmac, KeyInit, Mac};

use crate::{Body, Command, Desired, NodeId, Record, Version, Writer};

fn cmac(key: &[u8; 16], parts: &[&[u8]]) -> [u8; 16] {
    let mut mac = <Cmac<Aes128> as KeyInit>::new_from_slice(key).expect("AES-128 takes 16-byte keys");
    for part in parts {
        mac.update(part);
    }
    mac.finalize().into_bytes().into()
}

/// The controller's key, from the fleet key.
pub fn controller_key(fleet: &[u8; 16]) -> [u8; 16] {
    cmac(fleet, &[b"navgrid controller key"])
}

/// The MAC of a desired-state record with these fields.
pub fn desired_mac(key: &[u8; 16], origin: NodeId, version: Version, target: NodeId, command: &Command) -> [u8; 8] {
    let mut buf = [0u8; 64];
    let signed = postcard::to_slice(&(origin, version, target, command), &mut buf).map_or(&[][..], |b| &*b);
    let full = cmac(key, &[b"navgrid desired", signed]);
    let mut mac = [0u8; 8];
    mac.copy_from_slice(&full[..8]);
    mac
}

/// A new desired-state record from the controller's writer, with its MAC.
pub fn sign(writer: &mut Writer, key: &[u8; 16], target: NodeId, command: Command) -> Record {
    let version = writer.next();
    let mac = desired_mac(key, writer.origin(), version, target, &command);
    writer.record(Body::Desired(Desired { target, command, mac }))
}

/// Whether `record` is a desired-state record with the controller's MAC.
pub fn verify(key: &[u8; 16], record: &Record) -> bool {
    let Body::Desired(d) = &record.body else {
        return false;
    };
    desired_mac(key, record.origin, record.version, d.target, &d.command) == d.mac
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_signed_command_verifies_and_a_changed_one_does_not() {
        let key = controller_key(&[7; 16]);
        let mut writer = Writer::new(NodeId(0xc0), 3);
        let record = sign(&mut writer, &key, NodeId(5), Command::Blink { times: 2 });
        assert!(verify(&key, &record));
        assert!(!verify(&controller_key(&[8; 16]), &record));
        let mut forged = record.clone();
        if let Body::Desired(d) = &mut forged.body {
            d.target = NodeId(6);
        }
        assert!(!verify(&key, &forged));
        let mut replayed = record.clone();
        replayed.version.counter += 1;
        assert!(!verify(&key, &replayed));
    }
}
