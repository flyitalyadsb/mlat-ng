//! Bring the arrival times of one transmission, measured by different
//! receivers on unsynchronised clocks, into one common timebase.
//!
//! mlat-server walks a minimum spanning tree of the clock pairs: k-1 edges out
//! of up to k(k-1)/2, so every other pair's information is thrown away and one
//! bad edge silently shifts a whole subtree. Here every usable pair is an
//! equation c_i - c_j = theta_ij (c = per-receiver correction to the common
//! timebase), solved jointly by weighted least squares. Redundant edges average
//! out noise, an inconsistent edge shows up as a large residual and is dropped,
//! and the solution comes with its covariance, which the solver uses in full:
//! corrections that share edges have correlated errors.

use nalgebra::DMatrix;

use crate::geo::{dist, Ecef, C_AIR};

/// A receiver taking part in a group.
#[derive(Debug, Clone, Copy)]
pub struct Node {
    pub pos: Ecef,
    /// 1-sigma timestamp jitter of this receiver, seconds.
    pub jitter: f64,
}

/// One received copy: which node, and its local clock time in seconds.
#[derive(Debug, Clone, Copy)]
pub struct Copy {
    pub node: usize,
    pub s: f64,
}

#[derive(Debug, Clone)]
pub struct Normalized {
    /// Indices into the input `copies` that made it into the common timebase.
    pub copies: Vec<usize>,
    /// Arrival time of each of those copies in the common timebase, seconds.
    pub t: Vec<f64>,
    /// Covariance of `t`, seconds^2.
    pub cov: DMatrix<f64>,
    /// Edges used / dropped as inconsistent.
    pub edges: usize,
    pub dropped: usize,
}

/// Normalised residual (squared, in sigmas) above which an edge is treated as
/// inconsistent with the others and dropped.
const EDGE_OUTLIER2: f64 = 25.0;

struct Edge {
    i: usize,
    j: usize,
    theta: f64,
    var: f64,
}

fn components(n: usize, edges: &[Edge]) -> Vec<usize> {
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(p: &mut [usize], mut x: usize) -> usize {
        while p[x] != x {
            p[x] = p[p[x]];
            x = p[x];
        }
        x
    }
    for e in edges {
        let (a, b) = (find(&mut parent, e.i), find(&mut parent, e.j));
        if a != b {
            parent[a] = b;
        }
    }
    (0..n).map(|x| find(&mut parent, x)).collect()
}

/// `edge(i, j)` for node indices i < j returns (theta, var) with
/// s_j - s_i ~= theta for simultaneous events, evaluated at node i's time of
/// this group, or None when that pair is not usable.
pub fn normalize(
    nodes: &[Node],
    copies: &[Copy],
    mut edge: impl FnMut(usize, usize) -> Option<(f64, f64)>,
) -> Option<Normalized> {
    let n = nodes.len();
    if n < 2 || copies.len() < 2 {
        return None;
    }
    let mut edges = Vec::new();
    for i in 0..n {
        for j in (i + 1)..n {
            if let Some((theta, var)) = edge(i, j) {
                if var.is_finite() && var > 0.0 && theta.is_finite() {
                    edges.push(Edge { i, j, theta, var });
                }
            }
        }
    }
    let mut dropped = 0;
    loop {
        if edges.is_empty() {
            return None;
        }
        // Largest connected component of the usable-pair graph.
        let comp = components(n, &edges);
        let mut size = vec![0usize; n];
        for &c in &comp {
            size[c] += 1;
        }
        let best = (0..n).max_by_key(|&c| size[c]).unwrap();
        if size[best] < 2 {
            return None;
        }
        let members: Vec<usize> = (0..n).filter(|&x| comp[x] == best).collect();
        let mut slot = vec![usize::MAX; n];
        // Reference = best-connected member (keeps the system well conditioned).
        let mut degree = vec![0usize; n];
        for e in &edges {
            if comp[e.i] == best {
                degree[e.i] += 1;
                degree[e.j] += 1;
            }
        }
        let reference = *members.iter().max_by_key(|&&m| degree[m]).unwrap();
        let mut k = 0;
        for &m in &members {
            if m != reference {
                slot[m] = k;
                k += 1;
            }
        }
        // Normal equations of c_i - c_j = theta_ij, c_ref = 0.
        let mut a = DMatrix::<f64>::zeros(k, k);
        let mut b = nalgebra::DVector::<f64>::zeros(k);
        let used: Vec<&Edge> = edges.iter().filter(|e| comp[e.i] == best).collect();
        for e in &used {
            let w = 1.0 / e.var;
            let (si, sj) = (slot[e.i], slot[e.j]);
            if si != usize::MAX {
                a[(si, si)] += w;
                b[si] += w * e.theta;
            }
            if sj != usize::MAX {
                a[(sj, sj)] += w;
                b[sj] -= w * e.theta;
            }
            if si != usize::MAX && sj != usize::MAX {
                a[(si, sj)] -= w;
                a[(sj, si)] -= w;
            }
        }
        let chol = a.clone().cholesky()?;
        let c = chol.solve(&b);
        let cov_c = chol.inverse();
        let corr = |m: usize| if m == reference { 0.0 } else { c[slot[m]] };

        // Robustness: with redundant edges, drop the single worst outlier and
        // re-solve. A spanning tree (edges == members - 1) has zero residuals
        // by construction, so nothing can be learnt there.
        if used.len() + 1 > members.len() {
            let mut worst = None;
            let mut worst_r2 = EDGE_OUTLIER2;
            for (idx, e) in edges.iter().enumerate() {
                if comp[e.i] != best {
                    continue;
                }
                let r = corr(e.i) - corr(e.j) - e.theta;
                let r2 = r * r / e.var;
                if r2 > worst_r2 {
                    worst_r2 = r2;
                    worst = Some(idx);
                }
            }
            if let Some(idx) = worst {
                if dropped < 3 {
                    edges.swap_remove(idx);
                    dropped += 1;
                    continue;
                }
            }
        }

        // Arrival times of the copies whose receiver is in the component.
        let sel: Vec<usize> = (0..copies.len()).filter(|&q| comp[copies[q].node] == best).collect();
        if sel.len() < 2 {
            return None;
        }
        let m = sel.len();
        let mut t = Vec::with_capacity(m);
        let mut cov = DMatrix::<f64>::zeros(m, m);
        for (r, &q) in sel.iter().enumerate() {
            let node = copies[q].node;
            t.push(copies[q].s + corr(node));
            for (cidx, &q2) in sel.iter().enumerate() {
                let node2 = copies[q2].node;
                if node != reference && node2 != reference {
                    cov[(r, cidx)] = cov_c[(slot[node], slot[node2])];
                }
            }
            cov[(r, r)] += nodes[node].jitter * nodes[node].jitter;
        }
        return Some(Normalized { copies: sel, t, cov, edges: used.len(), dropped });
    }
}

/// Split normalised copies into clusters that can be the same transmission.
///
/// Identical bytes do not mean one transmission: DF11 all-call replies repeat
/// bit for bit, and so do Mode A/C replies, so one message group can hold
/// several transmissions. Two copies can come from one transmission only if
/// their time difference is within the light time between the receivers
/// (plus margin). Clusters with fewer than `min` distinct receivers are
/// dropped. Returns indices into `norm.copies` / `norm.t`.
pub fn clusters(norm: &Normalized, copies: &[Copy], nodes: &[Node], min: usize) -> Vec<Vec<usize>> {
    let mut order: Vec<usize> = (0..norm.t.len()).collect();
    order.sort_by(|&a, &b| norm.t[a].partial_cmp(&norm.t[b]).unwrap_or(std::cmp::Ordering::Equal));
    let mut taken = vec![false; order.len()];
    let mut out = Vec::new();
    for start in 0..order.len() {
        let first = order[start];
        if taken[first] {
            continue;
        }
        let mut cl = vec![first];
        taken[first] = true;
        for &cand in &order[start + 1..] {
            if taken[cand] {
                continue;
            }
            // Sorted: once the gap exceeds the largest plausible light time
            // (2 ms ~ 600 km) nothing later can join.
            if norm.t[cand] - norm.t[first] > 2.5e-3 {
                break;
            }
            let nc = copies[norm.copies[cand]].node;
            let ok = cl.iter().all(|&m| {
                let nm = copies[norm.copies[m]].node;
                if nm == nc {
                    return false;
                }
                let d = dist(&nodes[nm].pos, &nodes[nc].pos);
                let sigma = (norm.cov[(m, m)] + norm.cov[(cand, cand)]).sqrt();
                (norm.t[cand] - norm.t[m]).abs() <= (d * 1.05 + 1000.0) / C_AIR + 4.0 * sigma
            });
            if ok {
                cl.push(cand);
                taken[cand] = true;
            }
        }
        if cl.len() >= min {
            out.push(cl);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo::{llh_to_ecef, Llh};

    fn node(lat: f64, lon: f64) -> Node {
        Node { pos: llh_to_ecef(Llh { lat, lon, alt: 100.0 }), jitter: 0.5e-6 }
    }

    #[test]
    fn joint_solution_recovers_offsets() {
        // True corrections c (s); local times s_i = tau - c_i.
        let c = [0.0, 0.25, -3.5, 10.0];
        let nodes: Vec<Node> = (0..4).map(|i| node(45.0 + i as f64 * 0.3, 9.0)).collect();
        let tau = 1000.0;
        let copies: Vec<Copy> = (0..4).map(|i| Copy { node: i, s: tau - c[i] }).collect();
        // theta_ij = s_j - s_i = c_i - c_j, with a little noise on one edge
        let n = normalize(&nodes, &copies, |i, j| {
            let noise = if (i, j) == (0, 3) { 0.3e-6 } else { 0.0 };
            Some((c[i] - c[j] + noise, (0.3e-6f64).powi(2)))
        })
        .unwrap();
        assert_eq!(n.t.len(), 4);
        for w in n.t.windows(2) {
            assert!((w[0] - w[1]).abs() < 0.2e-6, "{:?}", n.t);
        }
    }

    #[test]
    fn drops_an_inconsistent_edge() {
        let c = [0.0, 1.0, 2.0, 3.0, 4.0];
        let nodes: Vec<Node> = (0..5).map(|i| node(45.0, 9.0 + i as f64 * 0.2)).collect();
        let copies: Vec<Copy> = (0..5).map(|i| Copy { node: i, s: 50.0 - c[i] }).collect();
        let n = normalize(&nodes, &copies, |i, j| {
            let bad = if (i, j) == (1, 3) { 20e-6 } else { 0.0 };
            Some((c[i] - c[j] + bad, (0.3e-6f64).powi(2)))
        })
        .unwrap();
        assert_eq!(n.dropped, 1);
        for w in n.t.windows(2) {
            assert!((w[0] - w[1]).abs() < 0.1e-6);
        }
    }

    #[test]
    fn largest_component_only() {
        let nodes: Vec<Node> = (0..4).map(|i| node(45.0, 9.0 + i as f64)).collect();
        let copies: Vec<Copy> = (0..4).map(|i| Copy { node: i, s: 1.0 }).collect();
        // 0-1-2 connected, 3 isolated
        let n = normalize(&nodes, &copies, |i, j| if j < 3 { Some((0.0, 1e-13)) } else { None }).unwrap();
        assert_eq!(n.copies, vec![0, 1, 2]);
    }

    #[test]
    fn clusters_split_repeated_transmissions() {
        let nodes: Vec<Node> = (0..3).map(|i| node(45.0, 9.0 + i as f64 * 0.1)).collect();
        // two transmissions 100 ms apart, each seen by all three receivers
        let mut copies = vec![];
        for tx in [0.0, 0.1] {
            for i in 0..3 {
                copies.push(Copy { node: i, s: 10.0 + tx + i as f64 * 10e-6 });
            }
        }
        let n = normalize(&nodes, &copies, |_, _| Some((0.0, 1e-14))).unwrap();
        let cl = clusters(&n, &copies, &nodes, 3);
        assert_eq!(cl.len(), 2);
        assert!(cl.iter().all(|c| c.len() == 3));
    }
}
