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
    pub point_count: usize,
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
    pub edge: usize,
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
    pub orientation: Quat,
    pub vertices: Vec<Vec3>,
    pub edges: Vec<HalfEdge>,
    pub faces: Vec<Face>,
    pub planes: Vec<Plane>,
}

//Collision direction is from a to b(first to second collider).

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
    let ab = b - a;
    let ac = c - a;
    let abc = ab.cross(ac);
    // if abc.dot(a) < 0.0 {
    //     abc = -abc;
    //  }
    let plane_distance = -abc.dot(a);
    //if plane_distance * plane_distance < best.length_squared() {
    let t = (Vec3::dot(abc, point) + plane_distance) / Vec3::dot(abc, abc);

    (point - t * abc, abc)
}

//takes a plane and a point that will be projected on the plane
//returns the projected point and the minimum distance from point and the plane
pub fn project_point_on_plane(plane: &Plane, point: Vec3) -> (Vec3, Vec3) {
    //if plane_distance * plane_distance < best.length_squared() {
    let t =
        (Vec3::dot(plane.normal, point) + plane.distance) / Vec3::dot(plane.normal, plane.normal);

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
    plane.normal.dot(point) + plane.distance
}

pub fn max_signed_distance(a_convex: &ConvexHull, b_convex: &ConvexHull) -> (f32, usize) {
    //
    let mut longest_distance = f32::MIN;
    let mut longest_index = 0;
    // TODO also add rotation
    let transform_a_to_local_b = a_convex.center - b_convex.center;

    let rotation_a_to_b = a_convex
        .orientation
        .mul_quat(b_convex.orientation.conjugate());

    for (index, a_plane) in a_convex.planes.iter().enumerate() {
        let plane_in_local_b = Plane {
            normal: rotation_a_to_b.mul_vec3(a_plane.normal),
            distance: a_plane.distance + transform_a_to_local_b.dot(-a_plane.normal),
            edge: 0,
        };
        let b_vertex = b_convex.get_support(&-plane_in_local_b.normal);
        let distance = signed_distance(&plane_in_local_b, b_vertex);
        if distance > longest_distance {
            longest_distance = distance;
            longest_index = index;
        }
    }
    (longest_distance, longest_index)
}

pub fn line_plane_intersect_point(start: Vec3, end: Vec3, plane: &Plane) -> Vec3 {
    let line_dir = end - start;
    let nd = plane.normal.dot(line_dir);
    let pn = start.dot(plane.normal);

    let t = (-plane.distance - pn) / nd;
    start + (line_dir * t)
}

pub fn clipping(a_convex: &ConvexHull, b_convex: &ConvexHull, index: usize) -> Vec<Contactpoint> {
    let transform_a_to_local_b = a_convex.center - b_convex.center;
    let reference_plane = a_convex.get_plane(index);
    let rotation_a_to_b = a_convex
        .orientation
        .mul_quat(b_convex.orientation.conjugate());
    //get incident face
    let incident_face_id =
        find_incident_face(b_convex, rotation_a_to_b.mul_vec3(reference_plane.normal));
    println!("reference normal: {}", reference_plane.normal);
    println!(
        "a convex center: {}, b convex center: {}",
        a_convex.center, b_convex.center
    );
    //check all clip planes aka the planes next to the reference plane
    let starting_edge = reference_plane.edge;
    let mut current_edge = reference_plane.edge;
    let mut all_contact_manifold: Vec<Contactpoint> = vec![];
    let mut all_contact_point = vec![];

    let incident_face = &b_convex.planes[incident_face_id];
    let first_edge = &b_convex.edges[incident_face.edge];
    /*   println!("first_edge id: {}", first_edge.origin);
      println!("incident id: {}", incident_face_id);
      println!("incident normal: {}", incident_face.normal);
    */
    let mut start_vertex = first_edge.origin;

    //starting edge index
    let starting_edge_index = incident_face.edge;
    //next edge index from current edge index
    let mut next_edge_index: usize = first_edge.next.try_into().unwrap();

    let mut end_vertex;

    loop {
        end_vertex = b_convex.edges[next_edge_index].origin;
        let a = b_convex.get_vertex(start_vertex.try_into().unwrap());
        let b = b_convex.get_vertex(end_vertex.try_into().unwrap());
        let mut a_inside = true;
        let mut b_inside = true;
        let mut last_point = Vec3::default();
        let mut clipped_point = Vec3::default();
        //println!("nexd-edge-index; {}", next_edge_index);
        //println!("a center: {}, b center: {}", a, b);

        loop {
            let get_clip_edge_index = a_convex.get_edge(current_edge).twin;
            let get_clip_edge = a_convex.get_edge(get_clip_edge_index as usize);
            let clipping_plane = Plane {
                normal: -rotation_a_to_b.mul_vec3(
                    a_convex
                        .get_plane(get_clip_edge.face.try_into().unwrap())
                        .normal,
                ),
                distance: -(a_convex
                    .get_plane(get_clip_edge.face.try_into().unwrap())
                    .distance)
                    + transform_a_to_local_b.dot(
                        a_convex
                            .get_plane(get_clip_edge.face.try_into().unwrap())
                            .normal,
                    ),
                edge: 0,
            };

            if signed_distance(&clipping_plane, a) >= 0.0 {
                //a is in front of plane
                if signed_distance(&clipping_plane, b) >= 0.0 {
                    //b is in front of plane
                    last_point = b;
                } else {
                    //a is in front b is behind
                    b_inside = false;
                    clipped_point = line_plane_intersect_point(a, b, &clipping_plane);
                }
            } else {
                //a is behind plane
                a_inside = false;
                if signed_distance(&clipping_plane, b) >= 0.0 {
                    //b is in front of plane
                    last_point = b;
                    clipped_point = line_plane_intersect_point(a, b, &clipping_plane);
                } else {
                    b_inside = false;
                }
            }

            current_edge = a_convex.get_edge(current_edge).next.try_into().unwrap();
            if current_edge == starting_edge {
                if b_inside {
                    //println!("last: {}", last_point);
                    all_contact_point.push(last_point);
                }
                if a_inside ^ b_inside {
                    //println!("clipped: {}", clipped_point);
                    all_contact_point.push(clipped_point);
                }
                break;
            }
        }

        if next_edge_index == starting_edge_index {
            break;
        }

        start_vertex = end_vertex;
        next_edge_index = b_convex.edges[next_edge_index].next.try_into().unwrap();
    }

    let mut all_possible_contact_point = vec![];
    //remove all points that are in front of the reference plane
    let refplane = Plane {
        normal: rotation_a_to_b.mul_vec3(reference_plane.normal),
        distance: (reference_plane.distance) + transform_a_to_local_b.dot(-reference_plane.normal),
        edge: 0,
    };
    for inner_loop in all_contact_point {
        let distance = signed_distance(&refplane, inner_loop);
        println!("distance: {}", distance);
        println!("point: {}", inner_loop);
        if distance < 0.0 {
            let point = Contactpoint {
                position: inner_loop - distance * rotation_a_to_b.mul_vec3(reference_plane.normal),
                penetration: distance.abs(),
            };
            println!("add point: {}", point.position);
            all_contact_manifold.push(point);
            all_possible_contact_point.push(inner_loop);
        }
    }
    //all_possible_contact_point
    all_contact_manifold
}

/// Clips the vertices of the face/plane of a convexhull against the clipping plane
pub fn sutherman_hogde_clipping(
    convex_hull: &ConvexHull,
    plane_index: usize,
    clipping_plane: &Plane,
) -> Vec<Vec3> {
    let incident_face = &convex_hull.planes[plane_index];
    let first_edge = &convex_hull.edges[incident_face.edge];

    let mut start_vertex = first_edge.origin;

    //starting edge index
    let starting_edge_index = incident_face.edge;
    //next edge index from current edge index
    let mut next_edge_index: usize = first_edge.next.try_into().unwrap();

    let mut end_vertex;
    let mut clipped_vertices: Vec<Vec3> = vec![];

    loop {
        end_vertex = convex_hull.edges[next_edge_index].origin;
        let a = convex_hull.get_vertex(start_vertex.try_into().unwrap());
        let b = convex_hull.get_vertex(end_vertex.try_into().unwrap());

        if signed_distance(clipping_plane, a) > 0.0 {
            //a is in front of plane
            if signed_distance(clipping_plane, b) > 0.0 {
                //b is in front of plane
                clipped_vertices.push(b);
            } else {
                //a is in front b is behimd
                clipped_vertices.push(line_plane_intersect_point(a, b, clipping_plane));
            }
        } else {
            //a is behind plane
            if signed_distance(clipping_plane, b) > 0.0 {
                //b is in front of plane
                clipped_vertices.push(b);
                clipped_vertices.push(line_plane_intersect_point(a, b, clipping_plane));
            }
        }

        if next_edge_index == starting_edge_index {
            break;
        }

        start_vertex = end_vertex;
        next_edge_index = convex_hull.edges[next_edge_index].next.try_into().unwrap();
    }
    clipped_vertices
}

pub fn query_edge_direction(a_convex: &ConvexHull, b_convex: &ConvexHull) -> (f32, usize, usize) {
    let mut longest_distance = f32::MIN;
    let mut longest_index1 = 0;
    let mut longest_index2 = 0;
    //TODO! also add rotation
    let transform_a_to_local_b = a_convex.center - b_convex.center;

    for ((index1, edge1), twin1) in a_convex
        .edges
        .iter()
        .step_by(2)
        .enumerate()
        .zip(a_convex.edges.iter().skip(1).step_by(2))
    {
        let p1 = transform_a_to_local_b + a_convex.get_vertex(edge1.origin.try_into().unwrap());
        let q1 = transform_a_to_local_b + a_convex.get_vertex(twin1.origin.try_into().unwrap());
        let e1 = q1 - p1;

        let u1 = a_convex.get_plane(edge1.face.try_into().unwrap()).normal;
        let v1 = a_convex.get_plane(twin1.face.try_into().unwrap()).normal;

        for ((index2, edge2), twin2) in b_convex
            .edges
            .iter()
            .step_by(2)
            .enumerate()
            .zip(b_convex.edges.iter().skip(1).step_by(2))
        {
            let p2 = b_convex.get_vertex(edge2.origin.try_into().unwrap());
            let q2 = b_convex.get_vertex(twin2.origin.try_into().unwrap());
            let e2 = q2 - p2;

            let u2 = b_convex.get_plane(edge2.face.try_into().unwrap()).normal;
            let v2 = b_convex.get_plane(twin2.face.try_into().unwrap()).normal;

            if is_minkowski_face(&u1, &v1, &-e1, &-u2, &-v2, &-e2) {
                let separation = rn_project(&p1, &e1, &p2, &e2, &transform_a_to_local_b);
                if separation > longest_distance {
                    longest_index1 = index1;
                    longest_index2 = index2;
                    longest_distance = separation;
                }
            }
        }
    }
    (longest_distance, longest_index1, longest_index2)
}

fn rn_project(p1: &Vec3, e1: &Vec3, p2: &Vec3, e2: &Vec3, c1: &Vec3) -> f32 {
    let e1_x_e2 = e1.cross(*e2);
    let k_tolerance = 0.005;

    let length = e1_x_e2.length();

    if length < k_tolerance * f32::sqrt(e1.length_squared() * e2.length_squared()) {
        return f32::MIN;
    }

    let mut normal = e1_x_e2 / length;
    if normal.dot(*p1 - *c1) < 0.0 {
        normal = -normal;
    }

    normal.dot(*p2 - *p1)
}

pub fn find_incident_face(incident_convex: &ConvexHull, normal: Vec3) -> usize {
    let mut most_anti_parallel = f32::MAX;
    let mut most_anit_parallel_index = 0;
    for (index, plane) in incident_convex.planes.iter().enumerate() {
        let dot = plane.normal.dot(normal);
        if dot < most_anti_parallel {
            most_anit_parallel_index = index;
            most_anti_parallel = dot;
        }
    }
    most_anit_parallel_index
}

pub fn find_possible_contact_points(
    a_convex: &ConvexHull,
    b_convex: &ConvexHull,
    index: usize,
) -> Vec<Contactpoint> {
    println!("a center: {}", a_convex.center);
    let transform_b_to_local_a = b_convex.center - a_convex.center;
    let reference_plane = a_convex.get_plane(index);
    //get incident face
    let incident_face = find_incident_face(b_convex, reference_plane.normal);
    println!("reference normal: {}", reference_plane.normal);
    //check all clip planes aka the planes next to the reference plane
    let starting_edge = reference_plane.edge;
    let mut current_edge = reference_plane.edge;
    let mut all_contact_point = vec![];
    let mut all_contact_manifold: Vec<Contactpoint> = vec![];

    loop {
        let get_clip_edge_index = a_convex.get_edge(current_edge).twin;
        let get_clip_edge = a_convex.get_edge(get_clip_edge_index as usize);

        let clipping_plane = Plane {
            normal: -a_convex
                .get_plane(get_clip_edge.face.try_into().unwrap())
                .normal,
            distance: -(a_convex
                .get_plane(get_clip_edge.face.try_into().unwrap())
                .distance
                + transform_b_to_local_a.dot(
                    a_convex
                        .get_plane(get_clip_edge.face.try_into().unwrap())
                        .normal,
                )),
            edge: 0,
        };

        all_contact_point.push(sutherman_hogde_clipping(
            b_convex,
            incident_face,
            &clipping_plane,
        ));
        current_edge = a_convex.get_edge(current_edge).next.try_into().unwrap();
        if current_edge == starting_edge {
            break;
        }
    }
    let mut all_possible_contact_point = vec![];
    //remove all points that are in front of the reference plane
    for outer_loop in all_contact_point {
        for inner_loop in outer_loop {
            let distance = signed_distance(reference_plane, inner_loop);
            if distance < 0.0 {
                let point = Contactpoint {
                    position: inner_loop,
                    penetration: distance.abs(),
                };
                all_contact_manifold.push(point);
                all_possible_contact_point.push(inner_loop);
            }
        }
    }
    //all_possible_contact_point
    all_contact_manifold
}

pub fn convex_convex_solver(a_convex: &ConvexHull, b_convex: &ConvexHull) -> ContactManifold<4> {
    let mut conatct_manifold: ContactManifold<4> = Default::default();

    //SAT with faces from convex a
    let (a_distance, a_index) = max_signed_distance(a_convex, b_convex);
    if a_distance > 0.0 {
        return conatct_manifold;
    }

    //SAT with faces from convex b
    let (b_distance, b_index) = max_signed_distance(b_convex, a_convex);
    if b_distance > 0.0 {
        return conatct_manifold;
    }

    let (edge_distance, index1, index2) = query_edge_direction(a_convex, b_convex);
    if edge_distance > 0.0 {
        return conatct_manifold;
    }
    if a_distance < edge_distance && b_distance < edge_distance {
        conatct_manifold.point_count = 1;
        return conatct_manifold;
    }

    if a_distance > b_distance {
        println!(
            "!!!!!!!!!!!!!!!!a distance; {}, b distance: {}",
            a_distance, b_distance
        );
        let mut contact_points = clipping(a_convex, b_convex, a_index);
        for x in contact_points.iter() {
            println!("{}", x.position);
        }
        if contact_points.len() > 4 {
            println!(
                "found {} collision points, reducde to 4",
                contact_points.len()
            );
            contact_points =
                reduce_contact_points(&contact_points, &a_convex.get_plane(a_index).normal);
        }

        conatct_manifold.point_count = contact_points.len();
        conatct_manifold.normal = a_convex.get_plane(a_index).normal;
        for (index, point) in contact_points.iter().enumerate() {
            conatct_manifold.points[index] = Contactpoint {
                position: b_convex.center + b_convex.orientation.mul_vec3(point.position),
                penetration: point.penetration,
            };
        }
    } else {
        let mut contact_points = clipping(b_convex, a_convex, b_index);
        for x in contact_points.iter() {
            println!("{}", x.position);
        }
        if contact_points.len() > 4 {
            println!(
                "found {} collision points, reducde to 4",
                contact_points.len()
            );
            for x in contact_points.iter().take(4) {
                println!("{}", x.position);
            }
            contact_points =
                reduce_contact_points(&contact_points, &b_convex.get_plane(b_index).normal);
        }

        conatct_manifold.point_count = contact_points.len();
        conatct_manifold.normal = b_convex.get_plane(b_index).normal;
        for (index, point) in contact_points.iter().enumerate() {
            conatct_manifold.points[index] = Contactpoint {
                position: a_convex.center + a_convex.orientation.mul_vec3(point.position),
                penetration: point.penetration,
            };
        }
    }
    conatct_manifold
}

pub fn reduce_contact_points(
    contact_points: &Vec<Contactpoint>,
    face_normal: &Vec3,
) -> Vec<Contactpoint> {
    //get first point
    let mut points = vec![Contactpoint::default(); 4];
    points[0] = contact_points[0];
    let index;
    (points[1], index) = furthest_point(contact_points, points[0].position);
    let mut remaining_points = Vec::with_capacity(contact_points.len() - 1);
    remaining_points.extend_from_slice(&contact_points[0..index]);
    remaining_points.extend_from_slice(&contact_points[index + 1..]);
    let mut largest_index = 0;
    let mut largest_area = 0.0;
    for (index, test_point) in remaining_points.iter().enumerate() {
        let area = (test_point.position - points[0].position)
            .cross(test_point.position - points[1].position)
            .dot(*face_normal);
        if area.abs() > largest_area {
            largest_area = area;
            largest_index = index;
        }
    }
    points[2] = remaining_points[largest_index];
    let last_index = remaining_points.len() - 1;
    remaining_points.swap(largest_index, last_index);
    if largest_area > 0.0 {
        largest_area = 0.0;
        //counterclockwise order

        for (index, point) in remaining_points.iter().take(last_index).enumerate() {
            let area = (point.position - points[1].position)
                .cross(point.position - points[2].position)
                .dot(*face_normal);
            if area < largest_area {
                largest_index = index;
                largest_area = area;
            }
            let area = (point.position - points[2].position)
                .cross(point.position - points[0].position)
                .dot(*face_normal);
            if area < largest_area {
                largest_index = index;
                largest_area = area;
            }
            let area = (point.position - points[0].position)
                .cross(point.position - points[1].position)
                .dot(*face_normal);
            if area < largest_area {
                largest_index = index;
                largest_area = area;
            }
        }
    } else {
        //clockwise order
        largest_area = 0.0;
        for (index, point) in remaining_points.iter().take(last_index).enumerate() {
            let area = (point.position - points[1].position)
                .cross(point.position - points[0].position)
                .dot(*face_normal);
            if area < largest_area {
                largest_index = index;
                largest_area = area;
            }
            let area = (point.position - points[2].position)
                .cross(point.position - points[1].position)
                .dot(*face_normal);
            if area < largest_area {
                largest_index = index;
                largest_area = area;
            }
            let area = (point.position - points[0].position)
                .cross(point.position - points[2].position)
                .dot(*face_normal);
            if area < largest_area {
                largest_index = index;
                largest_area = area;
            }
        }
    }
    points[3] = remaining_points[largest_index];
    points
}

pub fn furthest_point(points: &[Contactpoint], start_point: Vec3) -> (Contactpoint, usize) {
    let mut furthest_distance_squared = f32::MIN;
    let mut index = 0;
    for (i, point) in points.iter().enumerate() {
        let distance_squared = point.position.distance_squared(start_point);
        if distance_squared > furthest_distance_squared {
            furthest_distance_squared = distance_squared;
            index = i;
        }
    }
    (points[index], index)
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

fn get_support(points: &Vec<Vec3>, direction: &Vec3) -> Vec3 {
    let mut max_index = 0;
    let mut max_projection = f32::MIN;

    for (index, vertex) in points.iter().enumerate() {
        let projection = direction.dot(*vertex);
        if projection > max_projection {
            max_index = index;
            max_projection = projection;
        }
    }
    points[max_index]
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

    pub fn new(position: Vec3, vertices: Vec<Vec3>, axis: Vec3, angle: f32) -> ConvexHull {
        let mut x = ConvexHull {
            center: position,
            vertices: vertices.clone(),
            edges: Default::default(),
            faces: Default::default(),
            planes: create_planes_box(vertices),
            orientation: Quat::from_axis_angle(axis, angle),
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
            next: 17,
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
            next: 1,
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

        let mut distance = -normal.dot(self.vertices[0]);

        let mut plane = Plane {
            normal,
            distance,
            edge: 0,
        };
        self.planes.push(plane);

        //2.plane
        normal = (self.vertices[0] - self.vertices[2]).cross(self.vertices[6] - self.vertices[2]);

        distance = -normal.dot(self.vertices[2]);

        plane = Plane {
            normal,
            distance,
            edge: 7,
        };
        self.planes.push(plane);

        //3.plane
        normal = (self.vertices[4] - self.vertices[6]).cross(self.vertices[7] - self.vertices[6]);

        distance = -normal.dot(self.vertices[6]);

        plane = Plane {
            normal,
            distance,
            edge: 22,
        };
        self.planes.push(plane);

        //4.plane
        normal = (self.vertices[5] - self.vertices[7]).cross(self.vertices[3] - self.vertices[7]);

        distance = -normal.dot(self.vertices[7]);

        plane = Plane {
            normal,
            distance,
            edge: 23,
        };
        self.planes.push(plane);

        //5.plane
        normal = (self.vertices[1] - self.vertices[0]).cross(self.vertices[4] - self.vertices[0]);

        distance = -normal.dot(self.vertices[0]);

        plane = Plane {
            normal,
            distance,
            edge: 16,
        };
        self.planes.push(plane);

        //6.plane
        normal = (self.vertices[2] - self.vertices[3]).cross(self.vertices[7] - self.vertices[3]);

        distance = -normal.dot(self.vertices[3]);

        plane = Plane {
            normal,
            distance,
            edge: 19,
        };
        self.planes.push(plane);
    }
}

pub fn create_planes_box(vertices: Vec<Vec3>) -> Vec<Plane> {
    //1.plane

    let mut planes = Vec::with_capacity(6);

    let mut normal = (vertices[2] - vertices[0]).cross(vertices[1] - vertices[0]);

    let mut distance = -normal.dot(vertices[0]);

    let mut plane = Plane {
        normal,
        distance,
        edge: 0,
    };
    planes.push(plane);

    //2.plane
    normal = (vertices[0] - vertices[2]).cross(vertices[6] - vertices[2]);

    distance = -normal.dot(vertices[2]);

    plane = Plane {
        normal,
        distance,
        edge: 7,
    };
    planes.push(plane);

    //3.plane
    normal = (vertices[4] - vertices[6]).cross(vertices[7] - vertices[6]);

    distance = -normal.dot(vertices[6]);

    plane = Plane {
        normal,
        distance,
        edge: 22,
    };
    planes.push(plane);

    //4.plane
    normal = (vertices[5] - vertices[7]).cross(vertices[3] - vertices[7]);

    distance = -normal.dot(vertices[7]);

    plane = Plane {
        normal,
        distance,
        edge: 23,
    };
    planes.push(plane);

    //5.plane
    normal = (vertices[1] - vertices[0]).cross(vertices[4] - vertices[0]);

    distance = -normal.dot(vertices[0]);

    plane = Plane {
        normal,
        distance,
        edge: 16,
    };
    planes.push(plane);

    //6.plane
    normal = (vertices[2] - vertices[3]).cross(vertices[7] - vertices[3]);

    distance = -normal.dot(vertices[3]);

    plane = Plane {
        normal,
        distance,
        edge: 19,
    };
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
            orientation: Default::default(),
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
