use std::fs::File;
use std::io::{BufRead, BufReader};

use anyhow::Result;
use na::{Matrix4, Point3, Vector2, Vector3, U3};
use nalgebra as na;

// ビルダーパターン用構造体。
// モデル行列を受け付ける。
pub struct TriangleMeshBuilderModelMatrix {
    filename: String,
    model_matrix: Matrix4<f64>,
}

impl TriangleMeshBuilderModelMatrix {
    /// モデル行列をセットする。
    pub fn model_matrix(self, model: Matrix4<f64>) -> Self {
        Self {
            filename: self.filename,
            model_matrix: model,
        }
    }

    /// セットされたモデル行列とファイル名からobjを読み込んでTriangleMeshを構築する。
    pub fn build(self) -> Result<TriangleMesh> {
        // モデル行列。
        let model = self.model_matrix;

        // モデル行列をもとに法線を変換する行列を求めている。
        // 左上の3x3行列を取り出し、逆行列を計算し転置している。
        // モデル行列が正常ならば求まるはず。
        // [法線の変換の話 - 穴日記](http://raytracing.hatenablog.com/entry/20130325/1364229762)
        let normal_matrix = model
            .fixed_slice::<U3, U3>(0, 0)
            .try_inverse()
            .expect("Could not caluculate the inverse matrix of Model.")
            .transpose();

        // 読み込んだ情報をためていくVecを作成。
        let mut vs = Vec::<Point3<f64>>::new();
        let mut ns = Vec::<Vector3<f64>>::new();
        let mut vts = Vec::<Vector2<f64>>::new();
        let mut vi = Vec::<(usize, usize, usize)>::new();
        let mut ni = Vec::<(usize, usize, usize)>::new();
        let mut vti = Vec::<(usize, usize, usize)>::new();

        // objファイルを一行ずつパースしていく。
        for line in BufReader::new(File::open(self.filename)?).lines() {
            let line = line?;

            // 空行ならスキップする。
            if line == "" {
                continue;
            }

            // 行を空白でsplitする。
            let tmp: Vec<&str> = line.split_whitespace().collect();
            match tmp[0] {
                // 頂点位置
                "v" => {
                    let v: Vec<f64> = tmp[1..=3].iter().map(|s| s.parse().unwrap()).collect();
                    let v = Point3::new(v[0], v[1], v[2]);
                    vs.push(model.transform_point(&v));
                }
                // 頂点法線
                "vn" => {
                    let n: Vec<f64> = tmp[1..=3].iter().map(|s| s.parse().unwrap()).collect();
                    let n = Vector3::new(n[0], n[1], n[2]);
                    ns.push((normal_matrix * n).normalize());
                }
                // UV座標
                "vt" => {
                    let vt: Vec<f64> = tmp[1..=2].iter().map(|s| s.parse().unwrap()).collect();
                    let vt = Vector2::new(vt[0], vt[1]);
                    vts.push(vt);
                }
                // 面情報
                "f" => {
                    // 面情報は次のようなフォーマット。
                    // f 0/0/0 1/1/1 2/2/2
                    // 空白でsplitした後に、'/'でsplitして面を構築する頂点のindexを取得する。
                    let v0: Vec<usize> =
                        tmp[1].split('/').map(|s| s.parse().unwrap_or(0)).collect();
                    let v1: Vec<usize> =
                        tmp[2].split('/').map(|s| s.parse().unwrap_or(0)).collect();
                    let v2: Vec<usize> =
                        tmp[3].split('/').map(|s| s.parse().unwrap_or(0)).collect();
                    vi.push((v0[0] - 1, v1[0] - 1, v2[0] - 1));
                    ni.push((v0[2] - 1, v1[2] - 1, v2[2] - 1));
                    vti.push((v0[1] - 1, v1[1] - 1, v2[1] - 1));
                }
                _ => (),
            }
        }

        Ok(TriangleMesh {
            vertices: vs,
            normals: ns,
            uvs: vts,
            v_indices: vi,
            n_indices: ni,
            uv_indices: vti,
        })
    }
}

/// 三角形メッシュのジオメトリ情報を保持する構造体。
/// [single_triangle]を利用し三角形の頂点を指定して一つの三角形を含むTriangleMeshを作成できる。
/// [new]を利用しobjファイルをもとにTriangleMeshを作成できる。
///
/// [new]で読み込むobjファイルは頂点位置、頂点法線、UV、三角形面の情報を含む必要がある。
/// 三角形面以外の面には対応していない。
///
/// # Example
///
/// ```
/// let tm_builder = TriangleMesh::new("filename.obj");
/// let tm_builder = tm.model_matrix(Matrix4::new_translation(Vector3::new(10.0, 0.0, 0.0)));
/// let tm = tm_builder.build()?;
/// ```
pub struct TriangleMesh {
    pub vertices: Vec<Point3<f64>>,
    pub normals: Vec<Vector3<f64>>,
    pub uvs: Vec<Vector2<f64>>,
    pub v_indices: Vec<(usize, usize, usize)>,
    pub n_indices: Vec<(usize, usize, usize)>,
    pub uv_indices: Vec<(usize, usize, usize)>,
}

impl TriangleMesh {
    /// ファイル名を受け取ってビルダーオブジェクトを生成する。
    pub fn new(filename: &str) -> TriangleMeshBuilderModelMatrix {
        TriangleMeshBuilderModelMatrix {
            filename: filename.to_string(),
            model_matrix: Matrix4::identity(),
        }
    }

    /// 一枚の三角形のTriangleMeshを生成する。
    pub fn single_triangle(
        v0: Point3<f64>,
        v1: Point3<f64>,
        v2: Point3<f64>,
        n0: Vector3<f64>,
        n1: Vector3<f64>,
        n2: Vector3<f64>,
        uv0: Vector2<f64>,
        uv1: Vector2<f64>,
        uv2: Vector2<f64>,
    ) -> TriangleMesh {
        TriangleMesh {
            vertices: vec![v0, v1, v2],
            normals: vec![n0, n1, n2],
            uvs: vec![uv0, uv1, uv2],
            v_indices: vec![(0, 1, 2)],
            n_indices: vec![(0, 1, 2)],
            uv_indices: vec![(0, 1, 2)],
        }
    }
}
