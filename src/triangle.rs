use std::sync::Arc;

use na::{Point3, Vector3};
use nalgebra as na;

use super::material::Material;
use super::ray::Ray;
use super::triangle_mesh::TriangleMesh;

/// 三角形とrayのインターセクションの結果。
/// ヒットした場合はHitを、そうでない場合はNoneを。
#[derive(Clone)]
pub enum HitInfo {
    None,
    Hit {
        t: f64, // hit distance
        position: Point3<f64>,
        normal: Vector3<f64>,
        tangent: Vector3<f64>,
        bitangent: Vector3<f64>,
        material: Arc<Material>,
    },
}

impl HitInfo {
    /// 2つのHitInfoのうち近いHitを返す。
    pub fn closer(&self, other: &HitInfo) -> HitInfo {
        match (&self, &other) {
            (HitInfo::None, HitInfo::None) => self.clone(),
            (HitInfo::Hit { .. }, HitInfo::None) => self.clone(),
            (HitInfo::None, HitInfo::Hit { .. }) => other.clone(),
            (HitInfo::Hit { t: t0, .. }, HitInfo::Hit { t: t1, .. }) => {
                if t0 < t1 {
                    self.clone()
                } else {
                    other.clone()
                }
            }
        }
    }
}

/// Triangleの構造体。
/// TriangleMeshへの参照をArcで持つ。
#[derive(Clone)]
pub struct Triangle {
    pub index: usize,
    pub mesh: Arc<TriangleMesh>,
    material: Arc<Material>,
}

impl Triangle {
    /// TriangleMeshのindexを元にTriangleを作る
    pub fn new(index: usize, mesh: Arc<TriangleMesh>, material: Arc<Material>) -> Self {
        Self {
            index,
            mesh,
            material,
        }
    }

    /// Rayとのインターセクションを計算する。
    ///
    /// Singed Volumeで計算をする。
    /// 「signed volume ray triangle intersection」あたりでググればそれっぽいのが出てくるんじゃないかしら。
    ///
    /// intersectした位置と距離の他に線形補間した法線を計算し、
    /// UV座標系からtangent、bitangentを計算している。
    pub fn intersect(&self, ray: &Ray, t_min: f64, t_max: f64) -> HitInfo {
        let (via, vib, vic) = self.mesh.v_indices[self.index];
        let (nia, nib, nic) = self.mesh.n_indices[self.index];
        let (vta, vtb, vtc) = self.mesh.uv_indices[self.index];
        let a = self.mesh.vertices[via].coords;
        let b = self.mesh.vertices[vib].coords;
        let c = self.mesh.vertices[vic].coords;
        let na = self.mesh.normals[nia];
        let nb = self.mesh.normals[nib];
        let nc = self.mesh.normals[nic];
        let vta = self.mesh.uvs[vta];
        let vtb = self.mesh.uvs[vtb];
        let vtc = self.mesh.uvs[vtc];

        let o = ray.o;
        let q = ray.o + ray.d;

        let vc = (q - o).dot(&(b - o).cross(&(a - o)));
        let vb = (q - o).dot(&(a - o).cross(&(c - o)));
        let va = (q - o).dot(&(c - o).cross(&(b - o)));

        if vc > 0.0 && vb > 0.0 && va > 0.0 {
            let alpha = va / (va + vb + vc);
            let beta = vb / (va + vb + vc);
            let gamma = vc / (va + vb + vc);
            let pos = alpha * a + beta * b + gamma * c;
            let t = (pos - o).norm();
            if t >= t_min && t <= t_max {
                let delta_pos_1 = b - a;
                let delta_pos_2 = c - a;
                let delta_uv_1 = vtb - vta;
                let delta_uv_2 = vtc - vta;
                let r = 1.0 / (delta_uv_1.x * delta_uv_2.y - delta_uv_1.y * delta_uv_2.x);
                let tangent = (delta_pos_1 * delta_uv_2.y - delta_pos_2 * delta_uv_1.y) * r;
                let bitangent = (delta_pos_2 * delta_uv_1.x - delta_pos_1 * delta_uv_2.x) * r;

                HitInfo::Hit {
                    t,
                    position: Point3::from(pos),
                    normal: alpha * na + beta * nb + gamma * nc,
                    tangent,
                    bitangent,
                    material: Arc::clone(&self.material),
                }
            } else {
                HitInfo::None
            }
        } else {
            HitInfo::None
        }
    }
}
