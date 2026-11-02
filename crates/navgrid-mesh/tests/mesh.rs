use navgrid_mesh::session::{To, session};
use navgrid_mesh::*;

type Node = Replica<Fixed<96, 64, 32>>;

const CONTROLLER: NodeId = NodeId(0xc0_0000_0000);
const MINUTE: u64 = 60_000;

fn info() -> NodeInfo {
    NodeInfo {
        role: Role::Node,
        number: None,
        firmware: navgrid_proto::Firmware::new(0, 1, 0, 0, false),
        config: 0,
        hops: 0,
        frame: 0,
        neighbours: Default::default(),
    }
}

fn node(id: u64, sink: bool) -> Node {
    Replica::new(Config::new(NodeId(id), CONTROLLER, sink), 1, info(), 0)
}

/// A sink (id 1) and a chain of nodes behind it: 1 – 2 – 3 – …
fn chain(n: u64) -> Vec<Node> {
    (1..=n).map(|id| node(id, id == 1)).collect()
}

/// One round of sessions on every link of the chain, sink end first.
fn round(nodes: &mut [Node], now: u64) {
    for i in 0..nodes.len() - 1 {
        let (left, right) = nodes.split_at_mut(i + 1);
        session(&mut left[i], &mut right[0], now, 4096, |_, _, _| {});
    }
}

fn link(peer: u64, distance_mm: u32) -> Body {
    Body::Link(Link {
        peer: NodeId(peer),
        distance_mm,
        sigma_mm: 100,
        quality: 200,
        antenna_paths: 4,
        procedures: 10,
        failures: 0,
    })
}

fn health(battery_mv: u16) -> Body {
    Body::Health(Health {
        battery_mv,
        temperature_cdeg: 2500,
        uptime_s: 0,
        queued: 0,
    })
}

fn key(origin: u64, topic: Topic) -> Key {
    Key {
        origin: NodeId(origin),
        topic,
    }
}

fn version_at(r: &Node, k: Key) -> Option<Version> {
    r.get(&k).map(|s| s.record().version)
}

#[test]
fn only_the_controller_writes_desired_state_and_the_frame() {
    let c = CONTROLLER;
    let n = NodeId(5);
    let class = |origin, topic| Key { origin, topic }.class(c);
    assert_eq!(class(c, Topic::Frame), Some(Class::Global));
    assert_eq!(class(c, Topic::Position(n)), Some(Class::About(n)));
    assert_eq!(class(c, Topic::Desired(n, Setting::Blink)), Some(Class::Desired(n)));
    assert_eq!(class(c, Topic::Health), None);
    assert_eq!(class(n, Topic::Desired(n, Setting::Blink)), None);
    assert_eq!(class(n, Topic::Frame), None);
    assert_eq!(class(n, Topic::Position(n)), Some(Class::Near(n)));
    assert_eq!(class(n, Topic::Position(NodeId(6))), None);
    assert_eq!(class(n, Topic::Link(NodeId(6))), Some(Class::Near(n)));
    assert_eq!(class(n, Topic::Reported(Setting::Blink)), Some(Class::Up(n)));

    let mut a = node(5, false);
    let body = Body::Frame(Frame {
        version: 1,
        yaw_urad: 0,
        mirrored: false,
        offset: PointMm::default(),
    });
    assert_eq!(a.write(body, 0), Err(Error::Refused));
}

#[test]
fn newest_version_wins_in_any_order() {
    let mut writer = Writer::new(NodeId(7), 3);
    let old = writer.record(link(8, 1000));
    let new = writer.record(link(8, 2000));
    assert!(new.version > old.version);
    let k = old.key();

    let mut a = node(1, true);
    let mut b = node(1, true);
    assert_eq!(a.receive(old.clone(), 0), Ok(Received::New));
    assert_eq!(a.receive(new.clone(), 0), Ok(Received::Updated));
    assert_eq!(b.receive(new.clone(), 0), Ok(Received::New));
    assert_eq!(b.receive(old.clone(), 0), Ok(Received::Stale));
    assert_eq!(a.get(&k).unwrap().record(), b.get(&k).unwrap().record());

    // A later boot outranks everything before it.
    let mut rebooted = Writer::new(NodeId(7), 4);
    let after = rebooted.record(link(8, 500));
    assert!(after.version > new.version);
}

#[test]
fn neighbours_learn_each_other_and_two_hop_nodes() {
    // 1 (sink) – 2 – 3 – 4: node 2 keeps 3 and 4 (2 hops) but not 5.
    let mut nodes = chain(5);
    for t in 0..3 {
        round(&mut nodes, t * MINUTE);
    }
    let two = &nodes[1];
    assert_eq!(two.hops(), 1);
    assert_eq!(nodes[4].hops(), 4);
    assert!(two.in_neighbourhood(NodeId(3)));
    assert!(two.in_neighbourhood(NodeId(4)));
    assert!(!two.in_neighbourhood(NodeId(5)));
    assert!(two.get(&key(4, Topic::Node)).is_some());
    assert!(two.get(&key(5, Topic::Node)).is_none());
}

#[test]
fn links_spread_to_the_neighbourhood_and_reach_the_sink() {
    let mut nodes = chain(4);
    for t in 0..3 {
        round(&mut nodes, t * MINUTE);
    }
    nodes[3].write(link(3, 1234), 3 * MINUTE).unwrap();
    for t in 4..8 {
        round(&mut nodes, t * MINUTE);
    }
    let k = key(4, Topic::Link(NodeId(3)));
    let written = version_at(&nodes[3], k);
    assert_eq!(version_at(&nodes[2], k), written, "peer");
    assert_eq!(version_at(&nodes[1], k), written, "2 hops");
    assert_eq!(version_at(&nodes[0], k), written, "sink");
    // In custody only until handed downhill.
    assert!(nodes.iter().all(|n| n.outbox().iter().all(|p| p.record().key() != k)));
}

#[test]
fn health_goes_to_the_sink_only() {
    let mut nodes = chain(4);
    for t in 0..3 {
        round(&mut nodes, t * MINUTE);
    }
    nodes[3].write(health(2900), 3 * MINUTE).unwrap();
    for t in 4..8 {
        round(&mut nodes, t * MINUTE);
    }
    let k = key(4, Topic::Health);
    assert!(nodes[0].get(&k).is_some());
    assert!(nodes[1].get(&k).is_none());
    assert!(nodes[2].get(&k).is_none());
    // The relays learned the way back to node 4.
    assert_eq!(nodes[0].route(NodeId(4)), Some(NodeId(2)));
    assert_eq!(nodes[1].route(NodeId(4)), Some(NodeId(3)));
    assert_eq!(nodes[2].route(NodeId(4)), Some(NodeId(4)));
}

#[test]
fn a_command_reaches_its_target_and_the_report_comes_back() {
    let mut nodes = chain(5);
    for t in 0..6 {
        round(&mut nodes, t * MINUTE);
    }
    let mut controller = Writer::new(CONTROLLER, 1);
    let target = NodeId(5);
    let cmd = controller.record(Body::Desired(Desired {
        target,
        command: Command::Blink { times: 3 },
    }));
    let version = cmd.version;
    assert_eq!(nodes[0].inject(cmd, 6 * MINUTE), Ok(Received::New));

    let mut blinked = 0;
    for t in 7..20 {
        round(&mut nodes, t * MINUTE);
        let five = &mut nodes[4];
        let pending: Vec<_> = five.pending_commands().collect();
        for (v, command) in pending {
            assert_eq!(command, Command::Blink { times: 3 });
            blinked += 1;
            let report = Body::Reported(Reported {
                setting: Setting::Blink,
                applied: v,
                status: Status::Applied,
            });
            five.write(report, t * MINUTE).unwrap();
        }
    }
    assert_eq!(blinked, 1, "applied once");
    let report = nodes[0]
        .get(&key(5, Topic::Reported(Setting::Blink)))
        .expect("report at the sink");
    let Body::Reported(r) = &report.record().body else {
        panic!()
    };
    assert_eq!(r.applied, version);
    // The relays didn't keep the command: only the target and its peer do.
    let desired = Key {
        origin: CONTROLLER,
        topic: Topic::Desired(target, Setting::Blink),
    };
    assert!(nodes[1].get(&desired).is_none());
    assert!(nodes[2].get(&desired).is_none());
    assert!(nodes[3].get(&desired).is_some());
}

#[test]
fn a_relay_refuses_pushes_from_downhill() {
    let mut nodes = chain(3);
    for t in 0..3 {
        round(&mut nodes, t * MINUTE);
    }
    let sink = nodes[0].header();
    let mut writer = Writer::new(NodeId(9), 1);
    let r = writer.record(health(3000));
    assert_eq!(nodes[1].receive_push(&sink, r, 3 * MINUTE), Err(Error::Refused));
}

#[test]
fn records_expire_and_leave_with_their_scope() {
    let mut nodes = chain(3);
    for t in 0..3 {
        round(&mut nodes, t * MINUTE);
    }
    let k = key(3, Topic::Node);
    assert!(nodes[1].get(&k).is_some());
    // Node 3 goes silent: node 2 drops it once the peer times out.
    let later = 3 * MINUTE + u64::from(nodes[1].config().peer_timeout_ms) + 1;
    nodes[1].tick(later);
    assert!(!nodes[1].in_neighbourhood(NodeId(3)));
    assert!(nodes[1].get(&k).is_none());
    // The sink keeps it until it expires.
    assert!(nodes[0].get(&k).is_some());
    nodes[0].tick(later + u64::from(Ttl::default().node));
    assert!(nodes[0].get(&k).is_none());
}

#[test]
fn replicas_converge_after_a_partition() {
    // A ring of four non-sink nodes, split into pairs that then rejoin.
    let mut nodes: Vec<Node> = (1..=4).map(|id| node(id, false)).collect();
    let links = [(0, 1), (1, 2), (2, 3), (3, 0)];
    let run = |nodes: &mut [Node], links: &[(usize, usize)], t: u64| {
        for &(i, j) in links {
            let (lo, hi) = (i.min(j), i.max(j));
            let (left, right) = nodes.split_at_mut(hi);
            session(&mut left[lo], &mut right[0], t, 4096, |_, _, _| {});
        }
    };
    for t in 0..3 {
        run(&mut nodes, &links, t * MINUTE);
    }
    // Split {1, 2} | {3, 4}; each side writes.
    for (i, peer) in [(0, 2), (1, 1), (2, 4), (3, 3)] {
        nodes[i].write(link(peer, 1000 + i as u32), 3 * MINUTE).unwrap();
    }
    for t in 4..6 {
        run(&mut nodes, &[(0, 1), (2, 3)], t * MINUTE);
    }
    for t in 6..9 {
        run(&mut nodes, &links, t * MINUTE);
    }
    // Every node sees every link record, at the same version.
    for (i, peer) in [(0u64, 2u64), (1, 1), (2, 4), (3, 3)] {
        let k = key(i + 1, Topic::Link(NodeId(peer)));
        let v = version_at(&nodes[i as usize], k);
        assert!(v.is_some());
        for n in &nodes {
            assert_eq!(version_at(n, k), v, "node {:?} on {k:?}", n.id());
        }
    }
}

#[test]
fn in_sync_neighbours_exchange_only_summaries() {
    let mut nodes = chain(3);
    for t in 0..4 {
        round(&mut nodes, t * MINUTE);
    }
    let (left, right) = nodes.split_at_mut(2);
    let mut took = 0;
    let out = session(&mut left[1], &mut right[0], 5 * MINUTE, 4096, |_, _, _| took += 1);
    assert_eq!(took, 0);
    assert_eq!(out.a.sent.versions + out.a.sent.records + out.a.sent.pushes, 0);
    assert_eq!(out.b.sent.versions + out.b.sent.records + out.b.sent.pushes, 0);
    assert!(out.a.sent.summary > 0);
}

#[test]
fn the_budget_holds_records_back_for_the_next_session() {
    let mut nodes = chain(2);
    round(&mut nodes, 0);
    for peer in 10..20 {
        nodes[1].write(link(peer, 1000), MINUTE).unwrap();
    }
    let (left, right) = nodes.split_at_mut(1);
    let first = session(&mut left[0], &mut right[0], 2 * MINUTE, 120, |_, _, _| {});
    assert!(first.b.held_back);
    for t in 3..30 {
        let (left, right) = nodes.split_at_mut(1);
        session(&mut left[0], &mut right[0], t * MINUTE, 120, |_, _, _| {});
    }
    for peer in 10..20 {
        assert!(nodes[0].get(&key(2, Topic::Link(NodeId(peer)))).is_some());
    }
    assert!(nodes[1].outbox().is_empty());
}

#[test]
fn a_full_relay_keeps_pushes_with_their_sender() {
    let mut cfg = Config::new(NodeId(2), CONTROLLER, false);
    cfg.limits.outbox = 2;
    let mut nodes = vec![node(1, true), Replica::new(cfg, 1, info(), 0), node(3, false)];
    round(&mut nodes, 0);
    round(&mut nodes, MINUTE);
    // The sink goes away; node 3's records pile up at node 2 until its outbox is full.
    for i in 0..5 {
        nodes[2].write(health(3000 - i), (2 + i as u64) * MINUTE).unwrap();
        nodes[2]
            .write(link(10 + u64::from(i), 1000), (2 + i as u64) * MINUTE)
            .unwrap();
        let (left, right) = nodes.split_at_mut(2);
        session(&mut left[1], &mut right[0], (2 + i as u64) * MINUTE, 4096, |_, _, _| {});
    }
    assert_eq!(nodes[1].outbox().len(), 2);
    assert!(!nodes[2].outbox().is_empty(), "the sender kept what node 2 refused");
    // The sink comes back and everything drains.
    for t in 10..20 {
        round(&mut nodes, t * MINUTE);
    }
    assert!(nodes[1].outbox().is_empty() && nodes[2].outbox().is_empty());
    assert!(nodes[0].get(&key(3, Topic::Link(NodeId(14)))).is_some());
}

#[test]
fn sessions_report_what_each_side_took() {
    let mut nodes = chain(2);
    nodes[1].write(health(3000), 0).unwrap();
    let (left, right) = nodes.split_at_mut(1);
    let mut at_sink = Vec::new();
    session(&mut left[0], &mut right[0], MINUTE, 4096, |to, r, rx| {
        if to == To::A {
            at_sink.push((r.key().topic, rx));
        }
    });
    assert!(at_sink.contains(&(Topic::Health, Received::New)));
}

#[test]
fn pushes_go_commands_first_then_oldest_first() {
    let mut nodes = chain(2);
    round(&mut nodes, 0);
    let two = &mut nodes[1];
    two.write(health(3000), MINUTE).unwrap();
    two.write(link(7, 1000), 2 * MINUTE).unwrap();
    let report = Body::Reported(Reported {
        setting: Setting::Blink,
        applied: Version::default(),
        status: Status::Applied,
    });
    two.write(report, 3 * MINUTE).unwrap();
    // A newer health record keeps the older one's place in the queue.
    two.write(health(2990), 4 * MINUTE).unwrap();
    let sink = nodes[0].header();
    let mut order = Vec::new();
    nodes[1].pushes(&sink, 5 * MINUTE, |r| {
        order.push(r.key().topic);
        true
    });
    assert_eq!(
        order,
        [Topic::Reported(Setting::Blink), Topic::Health, Topic::Link(NodeId(7))]
    );
}
