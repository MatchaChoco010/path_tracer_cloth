use std::sync::Arc;
use std::time::Instant;

use na::Vector3;
use nalgebra as na;

use super::ray::Ray;
use super::triangle::{HitInfo, Triangle};

/// AABBの構造体。
/// centerはAABBをソートするのに利用している？
#[derive(Clone)]
pub struct AxisAlignBoundingBox {
    pub min: Vector3<f64>,
    pub max: Vector3<f64>,
    pub center: Vector3<f64>,
}

/// AABBとrayのオーバーラップ計算の結果。
enum Overlap {
    None,
    Hit { t: f64 },
}

impl AxisAlignBoundingBox {
    /// AABBの表面積を求めている。
    /// 直方体の各軸の辺の長さを計算し、長方形の面積を計算して足し合わせている。
    fn surface_area(&self) -> f64 {
        let x = self.max.x - self.min.x;
        let y = self.max.y - self.min.y;
        let z = self.max.z - self.min.z;
        2.0 * x * y + 2.0 * x * z + 2.0 * y * z
    }

    /// rayとaabbのオーバーラップの計算。
    fn overlap(&self, ray: &Ray) -> Overlap {
        let t_x1 = (self.max.x - ray.o.x) / ray.d.x;
        let t_x2 = (self.min.x - ray.o.x) / ray.d.x;
        let t_y1 = (self.max.y - ray.o.y) / ray.d.y;
        let t_y2 = (self.min.y - ray.o.y) / ray.d.y;
        let t_z1 = (self.max.z - ray.o.z) / ray.d.z;
        let t_z2 = (self.min.z - ray.o.z) / ray.d.z;

        let t_min = (t_x1.min(t_x2)).max(t_y1.min(t_y2)).max(t_z1.min(t_z2));
        let t_max = (t_x1.max(t_x2)).min(t_y1.max(t_y2)).min(t_z1.max(t_z2));

        if t_min <= t_max && (t_min > 0.0 || t_max > 0.0) {
            Overlap::Hit { t: t_min }
        } else {
            Overlap::None
        }
    }

    /// AABBを2つマージする。
    fn merge(&self, other: &Self) -> Self {
        let mut min = self.min.clone();
        let mut max = self.max.clone();
        min.x = min.x.min(other.min.x);
        min.y = min.y.min(other.min.y);
        min.z = min.z.min(other.min.z);
        max.x = max.x.max(other.max.x);
        max.y = max.y.max(other.max.y);
        max.z = max.z.max(other.max.z);
        let center = (min + max) / 2.0;
        Self { min, max, center }
    }
}

/// 三角形のAABBを計算する。
impl From<&Triangle> for AxisAlignBoundingBox {
    fn from(t: &Triangle) -> Self {
        let mut min = Vector3::new(f64::MAX, f64::MAX, f64::MAX);
        let mut max = Vector3::new(f64::MIN, f64::MIN, f64::MIN);
        let (i0, i1, i2) = t.mesh.v_indices[t.index];
        for v in vec![
            t.mesh.vertices[i0],
            t.mesh.vertices[i1],
            t.mesh.vertices[i2],
        ] {
            min.x = min.x.min(v.x);
            min.y = min.y.min(v.y);
            min.z = min.z.min(v.z);
            max.x = max.x.max(v.x);
            max.y = max.y.max(v.y);
            max.z = max.z.max(v.z);
        }
        let center = (min + max) / 2.0;
        Self { min, max, center }
    }
}

/// 三角形の集合のAABBを計算する。
impl From<&Vec<Triangle>> for AxisAlignBoundingBox {
    fn from(triangles: &Vec<Triangle>) -> Self {
        let mut min = Vector3::new(f64::MAX, f64::MAX, f64::MAX);
        let mut max = Vector3::new(f64::MIN, f64::MIN, f64::MIN);
        for t in triangles {
            let (i0, i1, i2) = t.mesh.v_indices[t.index];
            for v in vec![
                t.mesh.vertices[i0],
                t.mesh.vertices[i1],
                t.mesh.vertices[i2],
            ] {
                min.x = min.x.min(v.x);
                min.y = min.y.min(v.y);
                min.z = min.z.min(v.z);
                max.x = max.x.max(v.x);
                max.y = max.y.max(v.y);
                max.z = max.z.max(v.z);
            }
        }
        let center = (min + max) / 2.0;
        Self { min, max, center }
    }
}

/// AABBの集合のAABBを計算する。
impl From<Vec<AxisAlignBoundingBox>> for AxisAlignBoundingBox {
    fn from(aabbs: Vec<AxisAlignBoundingBox>) -> Self {
        let mut min = Vector3::new(f64::MAX, f64::MAX, f64::MAX);
        let mut max = Vector3::new(f64::MIN, f64::MIN, f64::MIN);
        for aabb in aabbs {
            min.x = min.x.min(aabb.min.x);
            min.y = min.y.min(aabb.min.y);
            min.z = min.z.min(aabb.min.z);
            max.x = max.x.max(aabb.max.x);
            max.y = max.y.max(aabb.max.y);
            max.z = max.z.max(aabb.max.z);
        }
        let center = (min + max) / 2.0;
        Self { min, max, center }
    }
}

/// splitの結果。
/// 分割後の最小コストが分割前より大きい場合分割は必要ないのでNoSplitを返す。
/// 分割後の最小コストが小さい場合はSplitで分割したあとの三角形群を返す。
enum SplitResult {
    NoSplit(Vec<Triangle>),
    Split(Vec<Triangle>, Vec<Triangle>),
}

/// BVHの構築で使う三角形群の分割の関数。
/// 三角形群を分割したほうがSAH関数の結果が小さくなる場合分割をする。
/// SAH (Surface Area Heuristic)については次のブログ記事などを参照のこと。
/// [bvh](https://shinjiogaki.github.io/bvh/)
fn split(triangles: Vec<Triangle>) -> SplitResult {
    if triangles.len() == 1 {
        return SplitResult::NoSplit(triangles);
    }

    // 定数はとりあえず1
    let cost_b = 1.0;
    let cost_o = 1.0;

    // 後で使うSplit前のAABBの表面積
    let parent_surface_area = AxisAlignBoundingBox::from(&triangles).surface_area();

    // 分割しない場合のコストを求める
    let cost_nosplit = triangles.len() as f64 * cost_o;

    // AABBと三角形をペアにする
    let aabbs = triangles
        .iter()
        .map(|t| AxisAlignBoundingBox::from(t))
        .collect::<Vec<_>>();
    let aabbs = aabbs.into_iter().zip(triangles.clone()).collect::<Vec<_>>();

    let splitx = {
        // X軸でソートする
        let mut aabbs_sorted = aabbs.clone();
        aabbs_sorted.sort_by(|(a, _), (b, _)| a.center.x.partial_cmp(&b.center.x).unwrap());

        // 順番にAABBを拡大しながら保持していく
        let mut sweep = Vec::with_capacity(aabbs_sorted.len());
        let mut aabb_ = aabbs_sorted[0].0.clone();
        for (aabb, _) in &aabbs_sorted.clone() {
            aabb_ = aabb_.merge(aabb);
            sweep.push(aabb_.clone());
        }
        // 2つに分離する関係上、三角形を全部含んだ場合は不要なので取り除く
        sweep.pop();

        // 逆順にAABBを拡大しながら保持していく
        let mut sweep_reverse = Vec::with_capacity(aabbs_sorted.len());
        let mut aabb_ = aabbs_sorted[aabbs_sorted.len() - 1].0.clone();
        for (aabb, _) in aabbs_sorted.iter().rev() {
            aabb_ = aabb_.merge(aabb);
            sweep_reverse.push(aabb_.clone());
        }
        // 2つに分離する関係上、三角形を全部含んだ場合は不要なので取り除く
        sweep_reverse.pop();

        // 昇順と降順にAABBを拡大したものをマージする
        // これでx軸で順に分割したあとの分割されたAABBのペアを作っている
        let sweep = sweep.into_iter().zip(sweep_reverse.into_iter().rev());

        let mut cost_min = f64::MAX;
        let mut split_pos = 0;
        for (i, (aabb0, aabb1)) in sweep.enumerate() {
            let i = i + 1;
            let surface_area0 = aabb0.surface_area();
            let surface_area1 = aabb1.surface_area();

            let cost = 2.0 * cost_b
                + surface_area0 / parent_surface_area * i as f64 * cost_o
                + surface_area1 / parent_surface_area * (triangles.len() - i) as f64 * cost_o;

            if cost < cost_min {
                cost_min = cost;
                split_pos = i;
            }
        }

        let mut split0 = aabbs_sorted.into_iter().map(|(_, t)| t).collect::<Vec<_>>();
        let split1 = split0.split_off(split_pos);
        (split0, split1, cost_min)
    };

    let splity = {
        // Y軸でソートする
        let mut aabbs_sorted = aabbs.clone();
        aabbs_sorted.sort_by(|(a, _), (b, _)| a.center.y.partial_cmp(&b.center.y).unwrap());

        // 順番にAABBを拡大しながら保持していく
        let mut sweep = Vec::with_capacity(aabbs_sorted.len());
        let mut aabb_ = aabbs_sorted[0].0.clone();
        for (aabb, _) in &aabbs_sorted.clone() {
            aabb_ = aabb_.merge(aabb);
            sweep.push(aabb_.clone());
        }
        // 2つに分離する関係上、三角形を全部含んだ場合は不要なので取り除く
        sweep.pop();

        // 逆順にAABBを拡大しながら保持していく
        let mut sweep_reverse = Vec::with_capacity(aabbs_sorted.len());
        let mut aabb_ = aabbs_sorted[aabbs_sorted.len() - 1].0.clone();
        for (aabb, _) in aabbs_sorted.iter().rev() {
            aabb_ = aabb_.merge(aabb);
            sweep_reverse.push(aabb_.clone());
        }
        // 2つに分離する関係上、三角形を全部含んだ場合は不要なので取り除く
        sweep_reverse.pop();

        // 昇順と降順にAABBを拡大したものをマージする
        let sweep = sweep.into_iter().zip(sweep_reverse.into_iter().rev());

        let mut cost_min = f64::MAX;
        let mut split_pos = 0;
        for (i, (aabb0, aabb1)) in sweep.enumerate() {
            let i = i + 1;
            let surface_area0 = aabb0.surface_area();
            let surface_area1 = aabb1.surface_area();

            let cost = 2.0 * cost_b
                + surface_area0 / parent_surface_area * i as f64 * cost_o
                + surface_area1 / parent_surface_area * (triangles.len() - i) as f64 * cost_o;

            if cost < cost_min {
                cost_min = cost;
                split_pos = i;
            }
        }

        let mut split0 = aabbs_sorted.into_iter().map(|(_, t)| t).collect::<Vec<_>>();
        let split1 = split0.split_off(split_pos);
        (split0, split1, cost_min)
    };

    let splitz = {
        // Z軸でソートする
        let mut aabbs_sorted = aabbs.clone();
        aabbs_sorted.sort_by(|(a, _), (b, _)| a.center.z.partial_cmp(&b.center.z).unwrap());

        // 順番にAABBを拡大しながら保持していく
        let mut sweep = Vec::with_capacity(aabbs_sorted.len());
        let mut aabb_ = aabbs_sorted[0].0.clone();
        for (aabb, _) in &aabbs_sorted.clone() {
            aabb_ = aabb_.merge(aabb);
            sweep.push(aabb_.clone());
        }
        // 2つに分離する関係上、三角形を全部含んだ場合は不要なので取り除く
        sweep.pop();

        // 逆順にAABBを拡大しながら保持していく
        let mut sweep_reverse = Vec::with_capacity(aabbs_sorted.len());
        let mut aabb_ = aabbs_sorted[aabbs_sorted.len() - 1].0.clone();
        for (aabb, _) in aabbs_sorted.iter().rev() {
            aabb_ = aabb_.merge(aabb);
            sweep_reverse.push(aabb_.clone());
        }
        // 2つに分離する関係上、三角形を全部含んだ場合は不要なので取り除く
        sweep_reverse.pop();

        // 昇順と降順にAABBを拡大したものをマージする
        let sweep = sweep.into_iter().zip(sweep_reverse.into_iter().rev());

        let mut cost_min = f64::MAX;
        let mut split_pos = 0;
        for (i, (aabb0, aabb1)) in sweep.enumerate() {
            let i = i + 1;
            let surface_area0 = aabb0.surface_area();
            let surface_area1 = aabb1.surface_area();

            let cost = 2.0 * cost_b
                + surface_area0 / parent_surface_area * i as f64 * cost_o
                + surface_area1 / parent_surface_area * (triangles.len() - i) as f64 * cost_o;

            if cost < cost_min {
                cost_min = cost;
                split_pos = i;
            }
        }

        let mut split0 = aabbs_sorted.into_iter().map(|(_, t)| t).collect::<Vec<_>>();
        let split1 = split0.split_off(split_pos);
        (split0, split1, cost_min)
    };

    // X, Y, Z軸のうちコストが最小のものを求める
    let (split0, split1, cost) = vec![splitx, splity, splitz]
        .into_iter()
        .min_by(|(_, _, c0), (_, _, c1)| c0.partial_cmp(c1).unwrap())
        .unwrap();

    // 分割後の最小コストと分割前のコストを比較して分割するかしないかを決めている
    if cost > cost_nosplit {
        SplitResult::NoSplit(triangles)
    } else {
        SplitResult::Split(split0, split1)
    }
}

/// BVH構造の要素。
/// LeafNodeにTriangleのリストが含まれている。
pub enum BVH {
    LeafNode {
        triangles: Vec<Triangle>,
        aabb: AxisAlignBoundingBox,
    },
    Node {
        nodes: Vec<Arc<BVH>>,
        aabb: AxisAlignBoundingBox,
    },
}

impl BVH {
    /// BVHの構築関数。
    /// 三角形のVecを受け取って構築したBVHを返す。
    pub fn new(triangles: Vec<Triangle>) -> Self {
        println!("Start building BVH...");
        let start = Instant::now();

        let bb = AxisAlignBoundingBox::from(&triangles);
        let bvh = BVH::subdivision(triangles, bb);

        let end = start.elapsed();
        println!(
            "Finish building BVH.\nBuild time: {}.{:03}s",
            end.as_secs(),
            end.subsec_millis()
        );
        bvh
    }

    // newで呼び出しているsubdivision関数
    // splitで分割する場合は再帰的に呼び出しを行いNodeを作成し、
    // 分割が必要ない場合はLeafNodeを作成している
    fn subdivision(triangles: Vec<Triangle>, space: AxisAlignBoundingBox) -> Self {
        let split_result = split(triangles);
        match split_result {
            SplitResult::NoSplit(triangles) => BVH::LeafNode {
                triangles,
                aabb: space,
            },
            SplitResult::Split(triangles0, triangles1) => {
                let space0 = AxisAlignBoundingBox::from(&triangles0);
                let space1 = AxisAlignBoundingBox::from(&triangles1);
                BVH::Node {
                    nodes: vec![
                        Arc::new(BVH::subdivision(triangles0, space0)),
                        Arc::new(BVH::subdivision(triangles1, space1)),
                    ],
                    aabb: space,
                }
            }
        }
    }

    /// BVHをtraverseする関数。
    /// hitを書き換えながら再帰呼び出しをするために。
    /// hitをmutな構造体としてtraverseをそのメソッドとして実装している。
    pub fn traverse(&self, ray: &Ray) -> (HitInfo, u32, u32) {
        let mut mut_hit = MutHit::new();
        mut_hit.traverse(ray, self);
        (
            mut_hit.hit,
            mut_hit.ray_traversal_count,
            mut_hit.ray_triangle_intersection_count,
        )
    }

    // aabbを返すメソッド
    fn aabb(&self) -> &AxisAlignBoundingBox {
        match self {
            BVH::LeafNode { aabb, .. } => aabb,
            BVH::Node { aabb, .. } => aabb,
        }
    }
}

/// hit情報を保持するグローバル変数代わりの構造体。
/// 再帰しながら書き換えるグローバル変数の代わりに構造体を用意する方法については次の記事を参照のこと。
/// [static mutの危険性 - Qiita](https://qiita.com/qnighy/items/46dbf8d2aff7c2531f4e)
struct MutHit {
    hit: HitInfo,
    ray_traversal_count: u32,
    ray_triangle_intersection_count: u32,
}

impl MutHit {
    // 初期値生成
    fn new() -> Self {
        Self {
            hit: HitInfo::None,
            ray_traversal_count: 0,
            ray_triangle_intersection_count: 0,
        }
    }

    // traverseの本体
    fn traverse(&mut self, ray: &Ray, node: &BVH) {
        self.ray_traversal_count += 1;

        // このノードのAABBとRayのオーバーラップの計算
        match node.aabb().overlap(ray) {
            // aabbとオーバーラップしていない場合は即リターン
            Overlap::None => return,
            // overlapしている場合で既存のhit情報と比較して打ち切りを入れる
            Overlap::Hit { t } => {
                if let HitInfo::Hit { t: t_existing, .. } = self.hit {
                    if t_existing < t {
                        return;
                    }
                }
            }
        }

        match node {
            // LeafNodeの場合
            BVH::LeafNode { triangles, .. } => {
                // 三角形とのインターセクションを計算する
                for t in triangles {
                    self.ray_triangle_intersection_count += 1;
                    self.hit = self.hit.closer(&t.intersect(ray, 0.0001, 1e12));
                }
            }
            // LeafNodeではない場合
            BVH::Node { nodes, .. } => {
                // nodesをaabb中心とrayの方向の符号付き距離で小さい順にソートする
                let mut nodes = nodes
                    .iter()
                    .map(|n| (n, n.aabb().center.dot(&ray.d)))
                    .collect::<Vec<_>>();
                nodes.sort_unstable_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap());
                let nodes = nodes.iter().map(|(node, _)| node);

                // 再帰的にtraverseを呼び出す
                for node in nodes {
                    self.traverse(ray, node)
                }
            }
        }
    }
}

impl BVH {
    /// BVHの情報を表示するメソッド。
    pub fn print_statistics(&self) {
        fn visit(bvh: &BVH) -> (u32, u32) {
            match bvh {
                BVH::LeafNode { .. } => (1, 1),
                BVH::Node { nodes, .. } => {
                    let (mut node_num_sum, mut leaf_node_num_sum) = (0, 0);
                    for (node_num, leaf_node_num) in nodes.iter().map(|sub_bvh| visit(sub_bvh)) {
                        node_num_sum += node_num;
                        leaf_node_num_sum += leaf_node_num;
                    }
                    (node_num_sum + 1, leaf_node_num_sum)
                }
            }
        }
        let (node, leaf_node) = visit(self);
        println!("Total number of nodes (include leaf nodes): {}", node);
        println!("Total number of leaf nodes: {}", leaf_node);
    }
}
