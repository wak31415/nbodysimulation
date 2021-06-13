/* Octree:
 *  ____ ____    y
 * | nw | ne |   ^
 * |____|____|   |
 * | sw | se |    --> x
 * |____|____|
 * 
 *  ______ ______    
 * |      |      |   z
 * | tsw  | tse  |   ^
 * |______|______|   |
 * |      |      |    --> x
 * | bsw  | bse  |
 * |______|______|
 * 
 * where the naming convention is:
 * t for top / b for bottom, top is towards +z
 * n for north / s for south, north is towards +y
 * e for east / w for west, east is towards +x
 */

pub trait Octree {
    fn center_of_mass(&self) -> Vector3<f32>;
    fn total_mass(&self) -> f32;
    fn side_length(&self) -> f32;
    
    fn population(&self) -> usize;
    // fn level(&self) -> usize;
    fn is_leaf(&self) -> bool;

    fn tnw(&self) -> Option<Box<Octree_member>>;
    fn tne(&self) -> Option<Box<Octree_member>>;
    fn tsw(&self) -> Option<Box<Octree_member>>;
    fn tse(&self) -> Option<Box<Octree_member>>;
    fn bnw(&self) -> Option<Box<Octree_member>>;
    fn bne(&self) -> Option<Box<Octree_member>>;
    fn bsw(&self) -> Option<Box<Octree_member>>;
    fn bse(&self) -> Option<Box<Octree_member>>;

    fn add(&self, &physics::Body) -> Octree_member;

    fn within_box(&self, &Vector3<f32>) -> bool;
}

pub enum Octree_member {
    Octree_node {
        geometric_center: Vector3<f32>
        center_of_mass: Vector3<f32>
        total_mass: f32
        side_length: f32

        population: usize
        tnw: Option<Box<Octree_member>>
        tne: Option<Box<Octree_member>>
        tsw: Option<Box<Octree_member>>
        tse: Option<Box<Octree_member>>
        bnw: Option<Box<Octree_member>>
        bne: Option<Box<Octree_member>>
        bsw: Option<Box<Octree_member>>
        bse: Option<Box<Octree_member>>
    },
    Octree_leaf {
        body: Option<Body>
        geometric_center: Vector3<f32>
        side_length: f32
    }
}

impl Octree for Octree_member::Octree_node {
    fn center_of_mass(&self) -> Vector3<f32> {
        self.center_of_mass
    }
    fn total_mass(&self) -> f32 {
        self.total_mass
    }
    fn side_length(&self) -> f32 {
        self.side_length
    }
    
    fn population(&self) -> usize {
        self.population
    }
    // fn level(&self) -> usize
    fn is_leaf(&self) -> bool {
        false
    }

    fn tnw(&self) -> Option<Box<Octree_member>> {
        self.tnw
    }
    fn tne(&self) -> Option<Box<Octree_member>> {
        self.tne
    }
    fn tsw(&self) -> Option<Box<Octree_member>> {
        self.tsw
    }
    fn tse(&self) -> Option<Box<Octree_member>> {
        self.tse
    }
    fn bnw(&self) -> Option<Box<Octree_member>> {
        self.bnw
    }
    fn bne(&self) -> Option<Box<Octree_member>> {
        self.bne
    }
    fn bsw(&self) -> Option<Box<Octree_member>> {
        self.bsw
    }
    fn bse(&self) -> Option<Box<Octree_member>> {
        self.bse
    }
}

impl Octree for Octree_member::Octree_leaf {
    fn center_of_mass(&self) -> Vector3<f32> {
        match self.body {
            None -> self.geometric_center
            Some(b) -> b.coordinates
        }
    }
    fn total_mass(&self) -> f32 {
        match self.body {
            None -> 0.0f32
            Some(b) -> b.mass
        }
    }
    fn side_length(&self) -> f32 {
        self.side_length
    }
    
    fn population(&self) -> usize {
        match self.body {
            None -> 0usize
            Some(_) -> 1usize
        }
    }
    // fn level(&self) -> usize
    fn is_leaf(&self) -> bool {
        true
    }

    fn tnw(&self) -> Option<Box<Octree_member>> {
        None
    }
    fn tne(&self) -> Option<Box<Octree_member>> {
        None
    }
    fn tsw(&self) -> Option<Box<Octree_member>> {
        None
    }
    fn tse(&self) -> Option<Box<Octree_member>> {
        None
    }
    fn bnw(&self) -> Option<Box<Octree_member>> {
        None
    }
    fn bne(&self) -> Option<Box<Octree_member>> {
        None
    }
    fn bsw(&self) -> Option<Box<Octree_member>> {
        None
    }
    fn bse(&self) -> Option<Box<Octree_member>> {
        None
    }

    fn add(&self, b: &physics::Body) -> Octree_member {

        match self.body {
            None -> Octree_member::Octree_leaf{
                body: b.clone()
                geometric_center: self.geometric_center
                side_length: self.side_length
            }
        }
    }
}

fn bounding_box_with_center(objects: &Vec<physics::Body>) -> (Vector3<f32>, Vector3<f32>, Vector3<f32>) {
    assert_ne!(objects.len, 0usize);
    let mut min_x: f32 = objects[0].coordinates[0];
    let mut min_y: f32 = objects[0].coordinates[1];
    let mut min_z: f32 = objects[0].coordinates[2];
    let mut max_x: f32 = objects[0].coordinates[0];
    let mut max_y: f32 = objects[0].coordinates[1];
    let mut max_z: f32 = objects[0].coordinates[2];
    
    for &obj in objects {
        if obj.coordinates[0] < min_x {
            min_x = obj.coordinates[0];
        }
        if obj.coordinates[1] < min_y {
            min_y = obj.coordinates[1];
        }
        if obj.coordinates[2] < min_z {
            min_z = obj.coordinates[2];
        }

        if obj.coordinates[0] < max_x {
            max_x = obj.coordinates[0];
        }
        if obj.coordinates[1] < max_y {
            max_y = obj.coordinates[1];
        }
        if obj.coordinates[2] < max_z {
            max_z = obj.coordinates[2];
        }
    }

    let bsw = Vector3::new(min_x, min_y, min_z);
    let tne = Vector3::new(max_x, max_y, max_z);

    return (
        bsw,
        tne,
        bounding_box_center(&bsw, &tne)
    );
}

fn bounding_box_center(min: &Vector3<f32>, max: &Vector3<f32>) -> Vector3<f32> {
    let x: f32 = (max[0] - min[0]) / 2.0f32;
    let y: f32 = (max[1] - min[1]) / 2.0f32;
    let z: f32 = (max[2] - min[2]) / 2.0f32;
    Vector3::new(x,y,z);
}

fn min_cube_side_len(min: &Vector3<f32>, max: &Vector3<f32>) -> f32 {
    let x_dist: f32 = max[0] - min[0];
    let y_dist: f32 = max[1] - min[1];
    let z_dist: f32 = max[2] - min[2];
    return max(x_dist, y_dist, z_dist);
}


pub fn create_octree_sequential(
    objects: &Vec<physics::Body>
) -> Box<Octree_member> {
    let (start_bsw, start_tne, start_center) = bounding_box_with_center(objects);
    let start_side_len = min_cube_side_len(&start_bsw, &start_tne);

    let mut root_node = Octree_member::Octree_leaf {
        body: None,
        geometric_center: start_center,
        side_length: start_side_len
    }

    for &obj in objects {
        let mut node = 
        
        obj.
    }
}