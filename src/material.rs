#![allow(non_snake_case)]
use nalgebra::Vector3;

/// 三角形のマテリアル。
/// ランバート反射、鏡面反射、透明、発光、布の5種類が実装されている。
pub enum Material {
    Lambert {
        kd: Vector3<f64>,
    },
    Mirror {
        ks: Vector3<f64>,
    },
    Glass {
        inner_eta: f64,
        outer_eta: f64,
    },
    Emissive {
        color: Vector3<f64>,
        radiance: f64,
    },
    Cloth {
        eta_0: f64,
        A_0: Vector3<f64>,
        kd_0: f64,
        gamma_s_0: f64,
        gamma_v_0: f64,
        alpha_0: f64,
        tangent_offset_u: Vec<f64>,

        eta_1: f64,
        A_1: Vector3<f64>,
        kd_1: f64,
        gamma_s_1: f64,
        gamma_v_1: f64,
        alpha_1: f64,
        tangent_offset_v: Vec<f64>,
    },
}

impl Material {
    /// Lambert平面。
    /// kdが反射係数。
    pub fn new_lambert(kd: Vector3<f64>) -> Self {
        Self::Lambert { kd }
    }

    /// 鏡面反射。
    /// ksが反射係数
    pub fn new_mirror(ks: Vector3<f64>) -> Self {
        Self::Mirror { ks }
    }

    /// 透明物体。
    /// 内側の屈折率[inner_eta]と外側の屈折率[outer_eta]を指定する。
    pub fn new_glass(inner_eta: f64, outer_eta: f64) -> Self {
        Self::Glass {
            inner_eta,
            outer_eta,
        }
    }

    /// 発行物体。
    /// 発光色[color]とその強さの放射輝度[radiance]を指定する。
    pub fn new_emissive(color: Vector3<f64>, radiance: f64) -> Self {
        Self::Emissive { color, radiance }
    }

    /// 布。
    /// 論文「A Practical Microcylinder Appearance Model for Cloth Rendering」を実装したもの。
    /// http://graphics.ucsd.edu/~henrik/papers/practical_microcylinder_appearance_model_for_cloth_rendering.pdf
    /// パラメータは論文を参照のこと。
    pub fn new_cloth(
        eta_0: f64,
        A_0: Vector3<f64>,
        kd_0: f64,
        gamma_s_0: f64,
        gamma_v_0: f64,
        alpha_0: f64,
        tangent_offset_u: Vec<f64>,

        eta_1: f64,
        A_1: Vector3<f64>,
        kd_1: f64,
        gamma_s_1: f64,
        gamma_v_1: f64,
        alpha_1: f64,
        tangent_offset_v: Vec<f64>,
    ) -> Self {
        Self::Cloth {
            eta_0,
            A_0,
            kd_0,
            gamma_s_0,
            gamma_v_0,
            alpha_0,
            tangent_offset_u,

            eta_1,
            A_1,
            kd_1,
            gamma_s_1,
            gamma_v_1,
            alpha_1,
            tangent_offset_v,
        }
    }
}
