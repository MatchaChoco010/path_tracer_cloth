#![allow(unused_variables)]
#![allow(non_snake_case)]

use std::f64::consts::PI;

use nalgebra::{Rotation3, Unit, Vector3};
use rand::distributions::{Distribution, Uniform};

use super::material::Material;
use super::ray::Ray;
use super::scene::Scene;
use super::triangle::HitInfo;

const MIN_DEPTH: u32 = 15;

/// パストレースの本体。
/// トラバーサルして反射Rayを計算して、というのをロシアンルーレットで止まるまで続ける。
/// 最低でもMIN_DEPTH回は再帰を飛ばす。
pub fn path_trace(ray: &Ray, scene: &Scene, depth: u32) -> Vector3<f64> {
    let mut rng = rand::thread_rng();
    let uniform = Uniform::new(0.0, 1.0);

    let (hit, _traversal_count, _intersection_count) = scene.bvh().traverse(&ray);
    match &hit {
        // 何にも当たらない場合は真っ黒を返す。
        HitInfo::None => Vector3::zeros(),
        // Hitした場合、その後の計算を続ける。
        HitInfo::Hit {
            material,
            position,
            normal,
            tangent,
            bitangent,
            ..
        } => {
            // ロシアンルーレットの確率。
            let russian_roulette_probability = if depth <= MIN_DEPTH {
                // MIN_DEPTHより現在のdepthが小さい場合は必ず次のRayを探索する。
                1.0
            } else {
                // depthがMIN_DEPTHを超えている場合、ロシアンルーレットにより次を探索するか決める。
                // ロシアンルーレットの確率は反射率のmaxを使うと良いと言われている。
                // 鏡面反射、ガラスの場合は再帰が必要なので必ず再帰する。
                // Emissiveの場合は放射輝度を返すためにロシアンルーレットは行わないため1.0。
                match material.as_ref() {
                    Material::Lambert { kd } => kd.x.max(kd.y).max(kd.z),
                    Material::Mirror { .. } => 1.0,
                    Material::Glass { .. } => 1.0,
                    Material::Emissive { .. } => 1.0,
                    Material::Cloth { A_0, A_1, .. } => {
                        A_0.x.max(A_0.y).max(A_0.z).max(A_1.x).max(A_1.y).max(A_1.z)
                    }
                }
            };

            // ロシアンルーレットで落ちた場合は黒を返す。
            if depth > MIN_DEPTH {
                if uniform.sample(&mut rng) >= russian_roulette_probability {
                    return Vector3::zeros();
                }
            }

            // マテリアルの種類によってパストレースの次のRayの計算の仕方が違うので分岐。
            match material.as_ref() {
                // ランバート反射
                Material::Lambert { kd } => {
                    let normal = if normal.dot(&ray.d) < 0.0 {
                        *normal
                    } else {
                        -normal
                    };

                    // cosine-weightedな確率密度関数を使って反射方向をサンプリングする。
                    let (u1, u2): (f64, f64) = (uniform.sample(&mut rng), uniform.sample(&mut rng));
                    let sample_dir = Vector3::new(
                        (2.0 * PI * u1).cos() * (1.0 - u2).sqrt(),
                        u2.sqrt(),
                        (2.0 * PI * u1).sin() * (1.0 - u2).sqrt(),
                    );
                    let up = if (normal.dot(&Vector3::y()) - 1.0).abs() <= 0.0001
                        || (normal.dot(&Vector3::y()) + 1.0).abs() <= 0.0001
                    {
                        Vector3::z()
                    } else {
                        Vector3::y()
                    };
                    let tangent_x = normal.cross(&up).normalize();
                    let tangent_z = tangent_x.cross(&normal).normalize();
                    let sample_dir =
                        tangent_x * sample_dir.x + normal * sample_dir.y + tangent_z * sample_dir.z;

                    // 反射方向のRay
                    let ray = Ray {
                        o: position.coords,
                        d: sample_dir,
                    };

                    // cosine-weightedなサンプリングなので、Lambert反射のコサイン項が吸収される。
                    kd.component_mul(&path_trace(&ray, scene, depth + 1))
                        / russian_roulette_probability
                }
                // 鏡面反射
                Material::Mirror { ks } => {
                    // 反射方向を求める。
                    let ray = Ray {
                        o: position.coords,
                        d: -2.0 * ray.d.dot(normal) * normal,
                    };

                    ks.component_mul(&path_trace(&ray, scene, depth + 1))
                        / russian_roulette_probability
                }
                // 透明物体
                Material::Glass {
                    inner_eta,
                    outer_eta,
                } => {
                    // 法線方向によってetaを入れ替えている。
                    let (eta_1, eta_2) = if ray.d.dot(normal) < 0.0 {
                        (outer_eta, inner_eta)
                    } else {
                        (inner_eta, outer_eta)
                    };

                    // 完全反射する場合があるのでその場合を検出している。
                    let is_total_internal =
                        (1.0 - (eta_1 / eta_2).powi(2) * (1.0 - (ray.d.dot(normal)).powi(2))) < 0.0;

                    if is_total_internal {
                        // 完全反射する場合
                        let ray_reflect = Ray {
                            o: position.coords,
                            d: -2.0 * ray.d.dot(normal) * normal + ray.d,
                        };

                        (path_trace(&ray_reflect, scene, depth + 1)) / russian_roulette_probability
                    } else {
                        // 完全反射しない場合rayの方向は2つ考えられる。
                        // refractするものとreflectするもののRayの方向を求めている。
                        let ray_refract = Ray {
                            o: position.coords,
                            d: (eta_1 / eta_2) * (ray.d - ray.d.dot(normal) * normal)
                                - (1.0
                                    - (eta_1 / eta_2).powi(2)
                                        * (1.0 - (ray.d.dot(normal)).powi(2)))
                                .sqrt()
                                    * normal,
                        };
                        let ray_reflect = Ray {
                            o: position.coords,
                            d: -2.0 * ray.d.dot(normal) * normal + ray.d,
                        };

                        let cos_theta_i = normal.dot(&(-ray.d));
                        let cos_theta_o = (-normal).dot(&ray_refract.d);
                        let rho_s = (eta_1 * cos_theta_i - eta_2 * cos_theta_o)
                            / (eta_1 * cos_theta_i + eta_2 * cos_theta_o);
                        let rho_p = (eta_1 * cos_theta_o - eta_2 * cos_theta_i)
                            / (eta_1 * cos_theta_o + eta_2 * cos_theta_i);
                        let fresnel = 0.5 * (rho_s.powi(2) + rho_p.powi(2));

                        let probability = 0.25 + 0.5 * fresnel;

                        if depth > 2 {
                            // 再帰深度が深い場合はreflectとrefractのどちらか一つを確立で選んで計算する。
                            // 確率にはフレネル係数を考慮する。
                            if uniform.sample(&mut rng) < fresnel {
                                path_trace(&ray_reflect, scene, depth + 1)
                                    / probability
                                    / russian_roulette_probability
                            } else {
                                path_trace(&ray_refract, scene, depth + 1)
                                    / (1.0 - probability)
                                    / russian_roulette_probability
                            }
                        } else {
                            // 再帰深度が2以下の浅い場合にはreflectとrefractの両方を計算して
                            // フレネル係数で足し合わせる。
                            (path_trace(&ray_reflect, scene, depth + 1) * fresnel
                                + path_trace(&ray_refract, scene, depth + 1) * (1.0 - fresnel))
                                / russian_roulette_probability
                        }
                    }
                }
                // 発光物質
                Material::Emissive { color, radiance } => {
                    // マテリアルの色と放射輝度を掛け合わせて返す
                    color * *radiance / russian_roulette_probability
                }
                // 布
                // 空気中の屈折率は1.0としている。
                // 論文通りの実装のはず。
                Material::Cloth {
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
                } => {
                    let normal = if normal.dot(&ray.d) < 0.0 {
                        *normal
                    } else {
                        -normal
                    };
                    let normal = normal.normalize();
                    let bitangent = normal.cross(&tangent.normalize()).normalize();
                    let tangent = bitangent.cross(&normal).normalize();

                    let (u1, u2): (f64, f64) = (uniform.sample(&mut rng), uniform.sample(&mut rng));
                    let wo = {
                        let phi = u1 * 2.0 * PI;
                        let cos_theta = 1.0 - u2;
                        let sin_theta = (1.0 - cos_theta * cos_theta).sqrt();
                        Vector3::new(phi.cos() * sin_theta, cos_theta, phi.sin() * sin_theta)
                    };
                    let wo = tangent * wo.x + normal * wo.y + bitangent * wo.z;
                    let wo = wo.normalize();
                    let wi = (-ray.d).normalize();

                    let mut Q = 0.0;
                    let mut u_value = Vector3::<f64>::zeros();
                    let mut v_value = Vector3::<f64>::zeros();

                    for tangent_offset in tangent_offset_u {
                        let v = bitangent.normalize();
                        let rot = Rotation3::from_axis_angle(
                            &Unit::new_normalize(v.clone()),
                            tangent_offset.to_radians(),
                        );
                        let u = rot * tangent.normalize();
                        let n = rot * normal;

                        let sin_theta_i = wi.dot(&u).min(1.0).max(-1.0);
                        let theta_i = sin_theta_i.asin();
                        let sin_theta_o = wo.dot(&u).min(1.0).max(-1.0);
                        let theta_o = sin_theta_o.asin();

                        let wi_on_normal = (wi - u * sin_theta_i).try_normalize(f64::EPSILON);
                        let wo_on_normal = (wo - u * sin_theta_o).try_normalize(f64::EPSILON);
                        if let None = wi_on_normal {
                            continue;
                        }
                        if let None = wo_on_normal {
                            continue;
                        }
                        let wi_on_normal = wi_on_normal.unwrap();
                        let wo_on_normal = wo_on_normal.unwrap();

                        let cos_phi_d = wi_on_normal.dot(&wo_on_normal).min(1.0).max(-1.0);
                        let phi_d = cos_phi_d.acos();
                        let theta_h = (theta_i + theta_o) * 0.5;
                        let theta_d = (theta_i - theta_o) * 0.5;

                        let cos_theta_o = theta_o.cos();

                        let wi_on_tangent_normal =
                            (wi - v * wi.dot(&v)).try_normalize(f64::EPSILON);
                        let wo_on_tangent_normal =
                            (wo - v * wo.dot(&v)).try_normalize(f64::EPSILON);
                        if let None = wi_on_tangent_normal {
                            continue;
                        }
                        if let None = wo_on_tangent_normal {
                            continue;
                        }
                        let wi_on_tangent_normal = wi_on_tangent_normal.unwrap();
                        let wo_on_tangent_normal = wo_on_tangent_normal.unwrap();

                        let cos_psi_d = wi_on_tangent_normal
                            .dot(&wo_on_tangent_normal)
                            .min(1.0)
                            .max(-1.0);
                        let psi_d = cos_psi_d.acos();

                        let cos_phi_i = n.dot(&wi_on_normal);
                        let cos_phi_o = n.dot(&wo_on_normal);
                        let cos_theta_i = theta_i.cos();
                        let cos_theta_o = theta_o.cos();
                        let cos_psi_i = n.dot(&wi_on_tangent_normal);
                        let cos_psi_o = n.dot(&wo_on_tangent_normal);

                        let eta_i = 1.0;
                        let Fr_cos_theta_i = theta_d.cos() * (phi_d * 0.5).cos();
                        let Fr = fresnel(*eta_0, eta_i, &wi, &n);
                        let fr_s =
                            Fr * (phi_d * 0.5).cos() * normalized_gaussian(*gamma_s_0, theta_h);

                        let Ft = 1.0 - Fr;
                        let F = Ft * Ft;
                        let fr_v = F
                            * ((1.0 - kd_0) * normalized_gaussian(*gamma_v_0, theta_h) + kd_0)
                            / (cos_theta_i + cos_theta_o)
                            * A_0;

                        let fs = (Vector3::new(fr_s, fr_s, fr_s) + fr_v) / theta_d.cos().powi(2);

                        let m_i = cos_phi_i.max(0.0);
                        let m_o = cos_phi_o.max(0.0);
                        let u = u_gaussian(phi_d);
                        let masking = (1.0 - u) * m_i * m_o + u * m_i.min(m_o);

                        let p_i = cos_psi_i.max(0.0);
                        let p_o = cos_psi_o.max(0.0);
                        let u = u_gaussian(psi_d);
                        let p = (1.0 - u) * p_i * p_o + u * p_i.min(p_o);

                        u_value += p * masking * fs * cos_theta_i;
                        Q += alpha_0 * p / tangent_offset_u.len() as f64;
                    }
                    u_value /= tangent_offset_u.len() as f64;

                    for tangent_offset in tangent_offset_v {
                        let v = tangent.normalize();
                        let rot = Rotation3::from_axis_angle(
                            &Unit::new_normalize(v.clone()),
                            tangent_offset.to_radians(),
                        );
                        let u = rot * bitangent.normalize();
                        let n = rot * normal;

                        let sin_theta_i = wi.dot(&u).min(1.0).max(-1.0);
                        let theta_i = sin_theta_i.asin();
                        let sin_theta_o = wo.dot(&u).min(1.0).max(-1.0);
                        let theta_o = sin_theta_o.asin();

                        let wi_on_normal = (wi - u * sin_theta_i).try_normalize(f64::EPSILON);
                        let wo_on_normal = (wo - u * sin_theta_o).try_normalize(f64::EPSILON);
                        if let None = wi_on_normal {
                            continue;
                        }
                        if let None = wo_on_normal {
                            continue;
                        }
                        let wi_on_normal = wi_on_normal.unwrap();
                        let wo_on_normal = wo_on_normal.unwrap();

                        let cos_phi_d = wi_on_normal.dot(&wo_on_normal).min(1.0).max(-1.0);
                        let phi_d = cos_phi_d.acos();
                        let theta_h = (theta_i + theta_o) * 0.5;
                        let theta_d = (theta_i - theta_o) * 0.5;
                        let cos_theta_o = theta_o.cos();

                        let wi_on_tangent_normal =
                            (wi - v * wi.dot(&v)).try_normalize(f64::EPSILON);
                        let wo_on_tangent_normal =
                            (wo - v * wo.dot(&v)).try_normalize(f64::EPSILON);
                        if let None = wi_on_tangent_normal {
                            continue;
                        }
                        if let None = wo_on_tangent_normal {
                            continue;
                        }
                        let wi_on_tangent_normal = wi_on_tangent_normal.unwrap();
                        let wo_on_tangent_normal = wo_on_tangent_normal.unwrap();

                        let cos_psi_d = wi_on_tangent_normal
                            .dot(&wo_on_tangent_normal)
                            .min(1.0)
                            .max(-1.0);
                        let psi_d = cos_psi_d.acos();

                        let cos_phi_i = n.dot(&wi_on_normal);
                        let cos_phi_o = n.dot(&wo_on_normal);
                        let cos_theta_i = theta_i.cos();
                        let cos_theta_o = theta_o.cos();
                        let cos_psi_i = n.dot(&wi_on_tangent_normal);
                        let cos_psi_o = n.dot(&wo_on_tangent_normal);

                        let eta_i = 1.0;
                        let Fr_cos_theta_i = theta_d.cos() * (phi_d * 0.5).cos();
                        let Fr = fresnel(*eta_0, eta_i, &wi, &n);
                        let fr_s =
                            Fr * (phi_d * 0.5).cos() * normalized_gaussian(*gamma_s_1, theta_h);

                        let Ft = 1.0 - Fr;
                        let F = Ft * Ft;
                        let fr_v = F
                            * ((1.0 - kd_1) * normalized_gaussian(*gamma_v_1, theta_h) + kd_1)
                            / (cos_theta_i + cos_theta_o)
                            * A_1;

                        let fs = (Vector3::new(fr_s, fr_s, fr_s) + fr_v) / theta_d.cos().powi(2);

                        let m_i = cos_phi_i.max(0.0);
                        let m_o = cos_phi_o.max(0.0);
                        let u = u_gaussian(phi_d);
                        let masking = (1.0 - u) * m_i * m_o + u * m_i.min(m_o);

                        let p_i = cos_psi_i.max(0.0);
                        let p_o = cos_psi_o.max(0.0);
                        let u = u_gaussian(psi_d);
                        let p = (1.0 - u) * p_i * p_o + u * p_i.min(p_o);

                        v_value += p * masking * fs * cos_theta_i;
                        Q += alpha_1 * p / tangent_offset_v.len() as f64;
                    }
                    v_value /= tangent_offset_v.len() as f64;

                    let mut fs = u_value * *alpha_0 + v_value * *alpha_1;
                    Q += (1.0 - alpha_0 - alpha_1) * wi.dot(&normal);
                    if Q > 0.0 {
                        fs /= Q;
                    }

                    let ray = Ray {
                        o: position.coords,
                        d: wo,
                    };
                    fs.component_mul(&path_trace(&ray, scene, depth + 1)) * (2.0 * PI)
                        / russian_roulette_probability
                }
            }
        }
    }
}

fn normalized_gaussian(beta: f64, theta: f64) -> f64 {
    (-theta * theta / (2.0 * beta * beta)).exp() / (2.0 * PI * beta * beta).sqrt()
}

fn fresnel(eta_t: f64, eta_i: f64, wi: &Vector3<f64>, n: &Vector3<f64>) -> f64 {
    let is_total_internal = (1.0 - (eta_i / eta_t).powi(2) * (1.0 - (wi.dot(n)).powi(2))) < 0.0;
    if is_total_internal {
        return 1.0;
    }
    let reflacted_dir = (eta_i / eta_t) * (wi - wi.dot(n) * n)
        - (1.0 - (eta_i / eta_t).powi(2) * (1.0 - (wi.dot(n)).powi(2))).sqrt() * n;
    let cos_theta_i = wi.dot(n);
    let cos_theta_o = reflacted_dir.dot(&-n);
    let rho_s =
        (eta_i * cos_theta_i - eta_t * cos_theta_o) / (eta_i * cos_theta_i + eta_t * cos_theta_o);
    let rho_p =
        (eta_i * cos_theta_o - eta_t * cos_theta_i) / (eta_i * cos_theta_o + eta_t * cos_theta_i);
    let fresnel = 0.5 * (rho_s.powi(2) + rho_p.powi(2));
    fresnel
}

fn u_gaussian(x: f64) -> f64 {
    let sd = 20.0f64.to_radians();
    (-x * x / (sd * sd * 2.0)).exp()
}
