use std::ops::Neg;

use crate::gjk::gjk;
mod gjk;
use bevy::{
    math::*,
    prelude::{Component, Transform},
};

#[derive(Default, Clone, Copy)]
pub struct Contactpoint {
    pub position: Vec3,
    pub penetration: f32,
}

pub struct ContactManifold<const COUNT: usize> {
    pub point_count: i8,
    pub normal: Vec3,
    pub points: [Contactpoint; COUNT],
}

impl<const COUNT: usize> Default for ContactManifold<COUNT> {
    fn default() -> Self {
        Self {
            point_count: Default::default(),
            normal: Default::default(),
            points: [Default::default(); COUNT],
        }
    }
}

#[derive(Component)]
pub struct Sphere {
    pub center: Vec3,
    pub radius: f32,
}

pub struct Plane {
    pub normal: Vec3,
    pub distance: f32,
}

#[derive(Component, Debug)]
pub struct Capsule {
    pub radius: f32,
    pub upper_center: Vec3,
    pub lower_center: Vec3,
}

#[derive(Default)]
pub struct HalfEdge {
    pub next: i8,
    pub twin: i8,
    pub origin: i8,
    pub face: i8,
}

pub struct Face {
    edge: i8,
}

#[derive(Component)]
pub struct ConvexHull {
    pub center: Vec3,
    pub vertices: Vec<Vec3>,
    pub edges: Vec<HalfEdge>,
    pub faces: Vec<Face>,
    pub planes: Vec<Plane>,
}

pub fn closet_point_on_segment(segment_a: Vec3, segment_b: Vec3, point: Vec3) -> Vec3 {
    let segment = segment_b - segment_a;
    let mut t = segment.dot(point - segment_a) / segment.dot(segment);
    t = f32::clamp(t, 0.0f32, 1.0f32);
    Vec3::lerp(segment_a, segment_b, t)
}

pub fn closet_point_segment_on_segment(
    segment_a: Vec3,
    segment_b: Vec3,
    segment_c: Vec3,
    segment_d: Vec3,
) -> (Vec3, Vec3) {
    let segment_dc = segment_d - segment_c;
    let line_dir_sqr_mag = segment_dc.dot(segment_dc);
    let in_plane_a = segment_a
        - ((Vec3::dot(segment_a - segment_c, segment_dc) / line_dir_sqr_mag) * segment_dc);
    let in_plane_b = segment_b
        - ((Vec3::dot(segment_b - segment_c, segment_dc) / line_dir_sqr_mag) * segment_dc);
    let in_plane_ba = in_plane_b - in_plane_a;
    let mut t =
        Vec3::dot(segment_c - in_plane_a, in_plane_ba) / Vec3::dot(in_plane_ba, in_plane_ba);
    t = if in_plane_a != in_plane_b { t } else { 0.0 };
    let seg_ab_to_line_cd = Vec3::lerp(segment_a, segment_b, f32::clamp(t, 0.0, 1.0));
    let seg_cd_to_seg_ab = closet_point_on_segment(segment_c, segment_d, seg_ab_to_line_cd);
    let seg_ab_to_seg_cd = closet_point_on_segment(segment_a, segment_b, seg_cd_to_seg_ab);
    (seg_cd_to_seg_ab, seg_ab_to_seg_cd)
}

pub fn sphere_sphere_overlap(sphere_a: &Sphere, sphere_b: &Sphere) -> bool {
    sphere_a.center.distance(sphere_b.center) - sphere_a.radius - sphere_b.radius <= 0.0
}

pub fn capsule_capsule_overlap(capsule_a: &Capsule, capsule_b: &Capsule) -> bool {
    let (first, second) = closet_point_segment_on_segment(
        capsule_a.lower_center,
        capsule_a.upper_center,
        capsule_b.lower_center,
        capsule_b.upper_center,
    );
    println!("distance {}", first.distance(second));
    first.distance(second) - capsule_a.radius - capsule_b.radius <= 0.0
}

pub fn capsule_convex_solver(capsule: &Capsule, convex: &ConvexHull) -> ContactManifold<1> {
    let (collision, closet_point_of_convex_to_cap, s, sim) = gjk(convex, capsule);
    if collision {
        println!("deep penetration");
    }
    let point = [Contactpoint {
        position: closet_point_of_convex_to_cap,
        penetration: closet_point_of_convex_to_cap.length() - capsule.radius,
    }];
    ContactManifold {
        point_count: 1,
        points: point,
        normal: closet_point_of_convex_to_cap,
    }
}

pub fn sphere_convex_overlap(a: &Sphere, b: &ConvexHull) -> (bool, usize, [Vec3; 4], Vec3) {
    //first use "gjk", we only need the closet point of the convex hull to the center point of the sphere
    let (collision, closet_point_of_convex_to_sphere, s, sim) = gjk(b, &a.center);
    (
        closet_point_of_convex_to_sphere.length() <= a.radius,
        s,
        sim,
        closet_point_of_convex_to_sphere + a.center,
    )
}

//takes three points that form the plane and a 4. point that will be projected on the plane
//returns the projected point and the minimum distance from 4. point and the plane
pub fn project_point_on_3points(a: Vec3, b: Vec3, c: Vec3, point: Vec3) -> (Vec3, Vec3) {
    let ab = a - b;
    let ac = a - c;
    let mut abc = ab.cross(ac);
    if abc.dot(a) < 0.0 {
        abc = -abc;
    }
    let plane_distance = abc.dot(a);
    //if plane_distance * plane_distance < best.length_squared() {
    let t = (Vec3::dot(abc, point) - plane_distance) / Vec3::dot(abc, abc);

    (point - t * abc, abc)
}

//takes a plane and a point that will be projected on the plane
//returns the projected point and the minimum distance from point and the plane
pub fn project_point_on_plane(plane: &Plane, point: Vec3) -> (Vec3, Vec3) {
    //if plane_distance * plane_distance < best.length_squared() {
    let t =
        (Vec3::dot(plane.normal, point) - plane.distance) / Vec3::dot(plane.normal, plane.normal);

    (point - t * plane.normal, plane.normal)
}

///Deep penetration is not working proper. The corners are not handled proper. Maybe check edges also for minimum distances
pub fn sphere_convex_solver(a: &Sphere, b: &ConvexHull) -> ContactManifold<1> {
    //first use "gjk", we only need the closet point of the convex hull to the center point of the sphere
    let (collision, closet_point_of_convex_to_sphere, s, sim) = gjk(b, &a.center);
    if collision {
        //deep penetration
        let mut distance = f32::MAX;
        let mut point_on_convex: Vec3;
        let mut best_point = Vec3::default();
        let mut best_normal = Vec3::default();
        let mut normal: Vec3;
        for plane in &b.planes {
            (point_on_convex, normal) =
                project_point_on_plane(plane, closet_point_of_convex_to_sphere + a.center);
            if point_on_convex.distance(a.center) < distance {
                distance = point_on_convex.distance(a.center);
                best_normal = normal;
                best_point = point_on_convex;
            }
        }

        let point = [Contactpoint {
            position: best_point,
            penetration: distance - a.radius,
        }];
        ContactManifold {
            point_count: 1,
            points: point,
            normal: best_normal,
        }
    } else if closet_point_of_convex_to_sphere.length() <= a.radius {
        //shallow penetration
        let point = [Contactpoint {
            position: closet_point_of_convex_to_sphere + a.center,
            penetration: closet_point_of_convex_to_sphere.length() - a.radius,
        }];
        ContactManifold {
            point_count: 2,
            points: point,
            normal: closet_point_of_convex_to_sphere,
        }
    } else {
        //no penetration
        ContactManifold {
            point_count: 0,
            points: [Contactpoint {
                position: closet_point_of_convex_to_sphere,
                penetration: 0.0,
            }],
            normal: closet_point_of_convex_to_sphere,
        }
    }
}

pub fn sphere_sphere_solver(a: &Sphere, b: &Sphere) -> ContactManifold<1> {
    let middle_origin = b.center.distance(a.center);
    let normal: Vec3 = (b.center - a.center) / middle_origin;
    let penetration = middle_origin - b.radius - a.radius;
    let point = [Contactpoint {
        position: normal * a.radius + a.center - penetration * 0.5f32,
        penetration,
    }];
    ContactManifold {
        point_count: 1,
        points: point,
        normal,
    }
}

pub fn sphere_capsule_overlap(a: &Sphere, b: &Capsule) -> bool {
    let point_on_segemnt = closet_point_on_segment(b.lower_center, b.upper_center, a.center);
    a.center.distance(point_on_segemnt) - a.radius - b.radius <= 0.0
}

pub fn sphere_capsule_solver(a: &Sphere, b: &Capsule, segment_point: Vec3) -> ContactManifold<1> {
    let normal = (segment_point - a.center) / (segment_point.distance(a.center));
    let penetration = segment_point.distance(a.center) - b.radius - a.radius;
    let point = [Contactpoint {
        position: normal * a.radius + a.center - penetration * 0.5f32,
        penetration,
    }];
    ContactManifold {
        point_count: 1,
        points: point,
        normal,
    }
}

pub fn signed_distance(plane: &Plane, point: Vec3) -> f32 {
    plane.normal.dot(point - (plane.normal * plane.distance))
}

pub fn max_signed_distance(a_convex: &ConvexHull, b_convex: &ConvexHull) -> (f32, usize) {
    //
    let mut longest_distance = f32::MIN;
    let mut longest_index = 0;
    //todo! also add rotation
    let transform_a_to_local_b = a_convex.center - b_convex.center;

    for (index, a_plane) in a_convex.planes.iter().enumerate() {
        let b_vertex = b_convex.get_support(&-a_plane.normal);
        let distance = signed_distance(a_plane, b_vertex + transform_a_to_local_b);
        if distance > longest_distance {
            longest_distance = distance;
            longest_index = index;
        }
    }
    (longest_distance, longest_index)
}

pub fn query_edge_direction(a_convex: &ConvexHull, b_convex: &ConvexHull) {
    // for edge in a_convex.edges {
    //   for edge in b_convex.edges {
    // if build_minkowski_face() {
    //    let sparation =
    // }
    //     }
    // }
}

pub fn convex_convex_solver(a_convex: &ConvexHull, b_convex: &ConvexHull) -> ContactManifold<4> {
    let mut conatct_manifold: ContactManifold<4> = Default::default();

    //SAT with faces from convex a
    let (longest_distance, index) = max_signed_distance(a_convex, b_convex);
    if longest_distance > 0.0 {
        return conatct_manifold;
    }

    //SAT with faces from convex b
    let (longest_distance, index) = max_signed_distance(b_convex, a_convex);
    if longest_distance > 0.0 {
        return conatct_manifold;
    }

    //SAT with edges from convex a and convex b
    conatct_manifold.point_count = 1;
    conatct_manifold
}

pub fn is_minkowski_face(a: &Vec3, b: &Vec3, bxa: &Vec3, c: &Vec3, d: &Vec3, dxc: &Vec3) -> bool {
    // Test if arcs AB and CD intersect on the unit sphere
    let cba = c.dot(*bxa);
    let dba = d.dot(*bxa);
    let adc = a.dot(*dxc);
    let bdc = b.dot(*dxc);

    cba * dba < 0.0 && adc * bdc < 0.0 && cba * bdc > 0.0
}

pub fn local_transform(a: &Mat3, b: &Mat3) -> Mat3 {
    let m = [
        a.col(0).dot(b.col(0)),
        a.col(1).dot(b.col(0)),
        a.col(2).dot(b.col(0)),
        a.col(0).dot(b.col(1)),
        a.col(1).dot(b.col(1)),
        a.col(2).dot(b.col(1)),
        a.col(0).dot(b.col(2)),
        a.col(1).dot(b.col(2)),
        a.col(2).dot(b.col(2)),
    ];
    Mat3::from_cols_array(&m)
}

pub trait GjkSupport {
    fn get_support(&self, direction: &Vec3) -> Vec3;
}

impl GjkSupport for Vec3 {
    fn get_support(&self, _direction: &Vec3) -> Vec3 {
        *self
    }
}

impl GjkSupport for Capsule {
    fn get_support(&self, direction: &Vec3) -> Vec3 {
        //closet_point_on_segment(self.lower_center, self.upper_center, *direction)
        let mut best_projection = self.lower_center;
        let projection_lower = direction.dot(self.lower_center);

        let projection_upper = direction.dot(self.upper_center);
        if projection_upper > projection_lower {
            best_projection = self.upper_center;
        }

        best_projection
    }
}

impl GjkSupport for ConvexHull {
    fn get_support(&self, direction: &Vec3) -> Vec3 {
        let mut max_index = 0;
        let mut max_projection = f32::MIN;

        for (index, vertex) in self.vertices.iter().enumerate() {
            let projection = direction.dot(*vertex);
            if projection > max_projection {
                max_index = index;
                max_projection = projection;
            }
        }
        self.vertices[max_index] + self.center
    }
}

impl ConvexHull {
    fn get_vertex(&self, index: usize) -> Vec3 {
        self.vertices[index]
    }

    fn get_edge(&self, index: usize) -> &HalfEdge {
        &self.edges[index]
    }

    fn get_face(&self, index: usize) -> &Face {
        &self.faces[index]
    }

    fn get_plane(&self, index: usize) -> &Plane {
        &self.planes[index]
    }

    fn get_support(&self, direction: &Vec3) -> Vec3 {
        let mut max_index = 0;
        let mut max_projection = f32::MIN;

        for (index, vertex) in self.vertices.iter().enumerate() {
            let projection = direction.dot(*vertex);
            if projection > max_projection {
                max_index = index;
                max_projection = projection;
            }
        }
        self.vertices[max_index]
    }

    pub fn new(position: Vec3, vertices: Vec<Vec3>) -> ConvexHull {
        let mut x = ConvexHull {
            center: position,
            vertices: vertices.clone(),
            edges: Default::default(),
            faces: Default::default(),
            planes: create_planes_box(vertices),
        };
        x.create_halfedges();
        x
    }

    pub fn create_halfedges(&mut self) {
        self.edges.resize_with(24, Default::default);

        self.edges[0] = HalfEdge {
            next: 2,
            twin: 1,
            origin: 0,
            face: 0,
        };
        self.edges[1] = HalfEdge {
            next: 13,
            twin: 0,
            origin: 1,
            face: 4,
        };

        self.edges[2] = HalfEdge {
            next: 4,
            twin: 3,
            origin: 1,
            face: 0,
        };
        self.edges[3] = HalfEdge {
            next: 1,
            twin: 2,
            origin: 3,
            face: 3,
        };

        self.edges[4] = HalfEdge {
            next: 6,
            twin: 5,
            origin: 3,
            face: 0,
        };
        self.edges[5] = HalfEdge {
            next: 21,
            twin: 4,
            origin: 2,
            face: 5,
        };

        self.edges[6] = HalfEdge {
            next: 0,
            twin: 7,
            origin: 2,
            face: 0,
        };
        self.edges[7] = HalfEdge {
            next: 8,
            twin: 6,
            origin: 0,
            face: 1,
        };

        self.edges[8] = HalfEdge {
            next: 10,
            twin: 9,
            origin: 2,
            face: 1,
        };
        self.edges[9] = HalfEdge {
            next: 5,
            twin: 8,
            origin: 6,
            face: 5,
        };

        self.edges[10] = HalfEdge {
            next: 12,
            twin: 11,
            origin: 6,
            face: 1,
        };
        self.edges[11] = HalfEdge {
            next: 18,
            twin: 10,
            origin: 4,
            face: 2,
        };

        self.edges[12] = HalfEdge {
            next: 7,
            twin: 13,
            origin: 4,
            face: 1,
        };
        self.edges[13] = HalfEdge {
            next: 14,
            twin: 12,
            origin: 0,
            face: 4,
        };

        self.edges[14] = HalfEdge {
            next: 16,
            twin: 15,
            origin: 4,
            face: 4,
        };
        self.edges[15] = HalfEdge {
            next: 11,
            twin: 14,
            origin: 5,
            face: 2,
        };

        self.edges[16] = HalfEdge {
            next: 17,
            twin: 17,
            origin: 5,
            face: 4,
        };
        self.edges[17] = HalfEdge {
            next: 23,
            twin: 16,
            origin: 1,
            face: 3,
        };

        self.edges[18] = HalfEdge {
            next: 22,
            twin: 19,
            origin: 6,
            face: 2,
        };
        self.edges[19] = HalfEdge {
            next: 9,
            twin: 18,
            origin: 7,
            face: 5,
        };

        self.edges[20] = HalfEdge {
            next: 3,
            twin: 21,
            origin: 7,
            face: 3,
        };
        self.edges[21] = HalfEdge {
            next: 19,
            twin: 20,
            origin: 3,
            face: 5,
        };

        self.edges[22] = HalfEdge {
            next: 15,
            twin: 23,
            origin: 7,
            face: 2,
        };
        self.edges[23] = HalfEdge {
            next: 20,
            twin: 22,
            origin: 5,
            face: 3,
        };
    }

    pub fn create_planes_box(&mut self) {
        self.planes.reserve(6);

        //1.plane
        let mut normal =
            (self.vertices[2] - self.vertices[0]).cross(self.vertices[1] - self.vertices[0]);

        let mut distance = normal.dot(self.vertices[0]);

        let mut plane = Plane { normal, distance };
        self.planes.push(plane);

        //2.plane
        normal = (self.vertices[0] - self.vertices[2]).cross(self.vertices[6] - self.vertices[2]);

        distance = normal.dot(self.vertices[2]);

        plane = Plane { normal, distance };
        self.planes.push(plane);

        //3.plane
        normal = (self.vertices[4] - self.vertices[6]).cross(self.vertices[7] - self.vertices[6]);

        distance = normal.dot(self.vertices[6]);

        plane = Plane { normal, distance };
        self.planes.push(plane);

        //4.plane
        normal = (self.vertices[5] - self.vertices[7]).cross(self.vertices[3] - self.vertices[7]);

        distance = normal.dot(self.vertices[7]);

        plane = Plane { normal, distance };
        self.planes.push(plane);

        //5.plane
        normal = (self.vertices[1] - self.vertices[0]).cross(self.vertices[4] - self.vertices[0]);

        distance = normal.dot(self.vertices[0]);

        plane = Plane { normal, distance };
        self.planes.push(plane);

        //6.plane
        normal = (self.vertices[2] - self.vertices[3]).cross(self.vertices[7] - self.vertices[3]);

        distance = normal.dot(self.vertices[3]);

        plane = Plane { normal, distance };
        self.planes.push(plane);
    }
}

pub fn create_planes_box(vertices: Vec<Vec3>) -> Vec<Plane> {
    //1.plane

    let mut planes = Vec::with_capacity(6);

    let mut normal = (vertices[2] - vertices[0]).cross(vertices[1] - vertices[0]);

    let mut distance = normal.dot(vertices[0]);

    let mut plane = Plane { normal, distance };
    planes.push(plane);

    //2.plane
    normal = (vertices[0] - vertices[2]).cross(vertices[6] - vertices[2]);

    distance = normal.dot(vertices[2]);

    plane = Plane { normal, distance };
    planes.push(plane);

    //3.plane
    normal = (vertices[4] - vertices[6]).cross(vertices[7] - vertices[6]);

    distance = normal.dot(vertices[6]);

    plane = Plane { normal, distance };
    planes.push(plane);

    //4.plane
    normal = (vertices[5] - vertices[7]).cross(vertices[3] - vertices[7]);

    distance = normal.dot(vertices[7]);

    plane = Plane { normal, distance };
    planes.push(plane);

    //5.plane
    normal = (vertices[1] - vertices[0]).cross(vertices[4] - vertices[0]);

    distance = normal.dot(vertices[0]);

    plane = Plane { normal, distance };
    planes.push(plane);

    //6.plane
    normal = (vertices[2] - vertices[3]).cross(vertices[7] - vertices[3]);

    distance = normal.dot(vertices[3]);

    plane = Plane { normal, distance };
    planes.push(plane);
    planes
}

struct FaceQuery {
    index: usize,
    separation: f32,
}

/*fn query_face_directions(
    out: &mut FaceQuery,
    transform1: &Transform,
    hull1: &ConvexHull,
    transform2: &Transform,
    hull2: &ConvexHull,
) {
    let transform = transform2.mul_transform(*transform1);
    let mut max_index = 0;
    let mut max_separation = f32::MIN;

    for (index, planes) in hull1.planes.iter().enumerate() {
        //let plane = transform;
        let separation = planes.
    }
}*/

fn distance(plane: &Plane, vec: &Vec3) -> f32 {
    plane.normal.dot(*vec) - plane.distance
}

fn project(plane: &Plane, hull: &ConvexHull) -> f32 {
    let support = hull.get_support(&-plane.normal);
    distance(plane, &support)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn convex_sphere_test() {
        let cube_vertices = vec![
            Vec3::new(-0.5, -0.5, -0.5),
            Vec3::new(-0.5, -0.5, 0.5),
            Vec3::new(-0.5, 0.5, -0.5),
            Vec3::new(-0.5, 0.5, 0.5),
            Vec3::new(0.5, -0.5, -0.5),
            Vec3::new(0.5, -0.5, 0.5),
            Vec3::new(0.5, 0.5, 0.5),
            Vec3::new(0.5, 0.5, -0.5),
        ];

        let triangle_vertices = vec![
            Vec3::new(-0.5, -0.5, 0.0),
            Vec3::new(-0.5, 0.5, 0.0),
            Vec3::new(0.5, 0.5, 0.0),
        ];

        let convex = ConvexHull {
            center: Vec3::new(0.0, 0.0, 0.0),
            vertices: cube_vertices,
            edges: Default::default(),
            faces: Default::default(),
            planes: Default::default(),
        };
        let point = Vec3::new(0.8416143, 0.0, 0.1);
        let s = Sphere {
            center: point,
            radius: 0.5,
        };
        let res = sphere_convex_overlap(&s, &convex);
        assert!(res.0)
    }

    #[test]
    fn sphere_capusle_test1() {
        let mut s = Sphere {
            center: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            radius: 1.0,
        };

        let mut c = Capsule {
            radius: 2.0,
            upper_center: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 1.0,
            },
            lower_center: Vec3 {
                x: 1.0,
                y: 2.0,
                z: 1.0,
            },
        };
        assert!(sphere_capsule_overlap(&s, &c));

        s.center = Vec3 {
            x: 4.0,
            y: 2.0,
            z: 2.0,
        };
        assert!(!sphere_capsule_overlap(&s, &c));

        c.lower_center = Vec3 {
            x: 4.0,
            y: 1.0,
            z: 1.0,
        };
        assert!(sphere_capsule_overlap(&s, &c));
    }
}
