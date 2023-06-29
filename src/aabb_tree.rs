use bevy::{
    math::*,
    prelude::{Component, Transform},
};

pub struct Tree {
    pub nodes: Vec<Node>,
    pub root_index: usize,
    pub free_list_entry: usize,
}

impl Tree {
    pub fn new() -> Tree {
        let v: Vec<Node> = Vec::with_capacity(16);
        let mut tree = Tree {
            nodes: v,
            root_index: usize::MAX,
            free_list_entry: 0,
        };
        tree.nodes.resize_with(16, Node::default);
        let size = tree.nodes.len() - 1;
        // Build a linked list for the free list.
        for i in 0..size {
            tree.nodes[i].parent_or_free.next = i + 1;
            tree.nodes[i].height = -1;
        }
        tree.nodes[size].parent_or_free.next = usize::MAX;
        tree.nodes[size].height = -1;
        tree
    }
    ///unsafe for accesing the union
    pub fn allocate_leaf_node(&mut self, object_index: usize) -> usize {
        if self.free_list_entry == usize::MAX {
            let size = self.nodes.len();
            self.nodes.resize_with(self.nodes.len() * 2, Node::default);
            let new_size = self.nodes.len() - 1;
            for i in size..new_size {
                self.nodes[i].parent_or_free.next = i + 1;
                self.nodes[i].height = -1;
            }

            self.nodes[new_size].parent_or_free.next = usize::MAX;
            self.nodes[new_size].height = -1;
            self.free_list_entry = size;
        }

        let node_id = self.free_list_entry;
        unsafe {
            self.free_list_entry = self.nodes[node_id].parent_or_free.next;
        }
        self.nodes[node_id].parent_or_free.parent = usize::MAX;
        self.nodes[node_id].child1 = usize::MAX;
        self.nodes[node_id].child2 = usize::MAX;
        self.nodes[node_id].height = 0;

        node_id
    }

    ///Push to be freed node in front of freelist.
    pub fn free_node(&mut self, node_id: usize) {
        self.nodes[node_id].parent_or_free.next = self.free_list_entry;
        self.nodes[node_id].height = -1;
        self.free_list_entry = node_id;
    }

    pub fn create_aabb(&mut self, aabb: Aabb) -> usize {
        let proxy_id = self.allocate_leaf_node(0);

        self.nodes[proxy_id].aabb_box.lower_bound = aabb.lower_bound;
        self.nodes[proxy_id].aabb_box.lower_bound = aabb.upper_bound;
        //self.nodes[proxyId].userData = userData;
        self.nodes[proxy_id].height = 0;
        //self.nodes[proxyId].moved = true;

        self.insert_leaf(proxy_id);

        proxy_id
    }

    pub fn insert_leaf(&mut self, leaf: usize) {
        if self.root_index == usize::MAX {
            self.root_index = leaf;
            self.nodes[leaf].parent_or_free.parent = usize::MAX;
            return;
        }
        // Find the best sibling for this node
        let leaf_aabb = &self.nodes[leaf].aabb_box;
        let index = self.root_index;
        while !self.nodes[index].is_leaf() {
            let child1 = self.nodes[index].child1;
            let child2 = self.nodes[index].child2;

            let area = self.nodes[index].aabb_box.surface_area();
            let combined_aabb = union(&self.nodes[index].aabb_box, leaf_aabb);
            let combined_area = combined_aabb.surface_area();

            // Cost of creating a new parent for this node and the new leaf
            let cost = 2.0 * combined_area;

            // Minimum cost of pushing the leaf further down the tree
            let inheritance_cost = 2.0 * (combined_area - area);
        }
    }
}

pub union Node_Type {
    parent: usize,
    next: usize,
}

impl Default for Node_Type {
    fn default() -> Self {
        Node_Type { parent: 0 }
    }
}

#[derive(Default)]
pub struct Node {
    pub aabb_box: Aabb,
    pub object_index: usize,
    pub parent_or_free: Node_Type,
    pub child1: usize,
    pub child2: usize,
    // leaf = 0, free node = -1
    pub height: i16,
}

impl Node {
    pub fn is_leaf(&self) -> bool {
        self.height == 0
    }
}

#[derive(Default)]
pub struct Aabb {
    pub lower_bound: Vec3,
    pub upper_bound: Vec3,
}

impl Aabb {
    pub fn min(&self, rhs: &Aabb) -> Vec3 {
        Vec3::new(
            if self.lower_bound.x < rhs.lower_bound.x {
                self.lower_bound.x
            } else {
                rhs.lower_bound.x
            },
            if self.lower_bound.y < rhs.lower_bound.y {
                self.lower_bound.y
            } else {
                rhs.lower_bound.y
            },
            if self.lower_bound.z < rhs.lower_bound.z {
                self.lower_bound.z
            } else {
                rhs.lower_bound.z
            },
        )
    }

    pub fn max(&self, rhs: &Aabb) -> Vec3 {
        Vec3 {
            x: if self.lower_bound.x > rhs.lower_bound.x {
                self.lower_bound.x
            } else {
                rhs.lower_bound.x
            },
            y: if self.lower_bound.y > rhs.lower_bound.y {
                self.lower_bound.y
            } else {
                rhs.lower_bound.y
            },
            z: if self.lower_bound.z > rhs.lower_bound.z {
                self.lower_bound.z
            } else {
                rhs.lower_bound.z
            },
        }
    }

    pub fn new(lower: Vec3, upper: Vec3) -> Aabb {
        Aabb {
            lower_bound: lower,
            upper_bound: upper,
        }
    }
    //computes the surfacearea of a 3D AABB
    //SA(a) short for surface area of a
    pub fn surface_area(&self) -> f32 {
        let width = self.upper_bound - self.lower_bound;
        2.0 * (width.x * width.y + width.x * width.z + width.y * width.z)
    }
}

//add simd stuff. cool stuff to benchmark later.
// C = a U b;   U is the cup notation for the union of a and b
//(all points of b and a are in the union)
pub fn union(a: &Aabb, b: &Aabb) -> Aabb {
    Aabb::new(a.min(b), a.max(b))
}

//computes the surfacearea of a 3D AABB
//SA(a) short for surface area of a
pub fn surface_area(a: &Aabb) -> f32 {
    let width = a.upper_bound - a.lower_bound;
    2.0 * (width.x * width.y + width.x * width.z + width.y * width.z)
}
