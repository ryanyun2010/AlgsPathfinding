#![allow(warnings)]
use std::{cmp::Ordering, collections::{BTreeSet, HashMap, HashSet}, time::Instant};

use rand::Rng;


/* Eight Movement Directions, Diagonal movements cost sqrt(2) */
#[derive(Debug,Clone)]
struct Node {
    parent: Option<[usize;2]>,
    cost: f32,
    hueristic_dist: f32,
    f: f32,
    position: [usize;2]
}

impl Node {
    pub fn new(parent: Option<[usize;2]>, cost: f32,position:[usize;2], goal: [usize;2]) -> Node {
        let h = ((goal[0] as f32 - position[0] as f32).powf(2.) + (goal[1] as f32 - position[1] as f32).powf(2.)).sqrt();

        Node {
            parent,
            cost,
            hueristic_dist: h   ,
            f: cost + h * 1.0001,
            position
        }
    }
}

#[derive(Debug,Clone)]
struct NodeD {
    parent: Option<[usize;2]>,
    cost: f32,
    position: [usize;2]
}
impl NodeD {
    pub fn new(parent: Option<[usize;2]>, cost: f32, position:[usize;2]) -> NodeD {
        NodeD {parent,cost,position}
    }
}
impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.f.partial_cmp(&other.f)
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(&other).unwrap()
    }
}
impl PartialEq for Node {
    fn eq(&self, other:&Self) -> bool {
        return self.position == other.position;
    }
}
impl Eq for Node {}

impl PartialEq for NodeD {
    fn eq(&self, other:&Self) -> bool {
        return self.position == other.position;
    }
}
impl Eq for NodeD {}


impl PartialOrd for NodeD {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.cost.partial_cmp(&other.cost)
    }
}

impl Ord for NodeD {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(&other).unwrap()
    }
}
struct Map {
    map: HashMap<[usize;2],bool>,
    nodes: HashMap<[usize;2], Node>, 
    nodes_d: HashMap<[usize;2], NodeD>, 
}

#[derive(Debug)]
enum Dir {
    Left,
    Right,
    Up,
    Down,
    LeftUp,
    RightUp,
    LeftDown,
    RightDown,
    None
}



impl Map {

    pub fn new(map: HashMap<[usize;2],bool>) -> Self {
        Map {
            map,
            nodes: HashMap::new(),
            nodes_d: HashMap::new(),
        }
    }


    /************************************************************************************************************************************************************************
     **************************************************************************** JUMP POINT SEARCH ************************************************************************
    ************************************************************************************************************************************************************************/
    fn prune(&self, cur_node: [usize; 2], parent: Option<[usize;2]>) -> (Vec<[usize;2]>, Vec<[usize;2]>) {
            if parent.is_none() {return (vec![
                [cur_node[0] - 1, cur_node[1]],
                [cur_node[0] +1, cur_node[1]],
                [cur_node[0] - 1, cur_node[1] - 1],
                [cur_node[0] +1, cur_node[1] - 1],
                [cur_node[0] - 1, cur_node[1] + 1],
                [cur_node[0] +1, cur_node[1] + 1],
                [cur_node[0], cur_node[1] + 1],
                [cur_node[0], cur_node[1] - 1]
            ],vec![]);}
            else {
                let parent_pos = parent.unwrap(); 
                    let dx = cur_node[0] as i32 - parent_pos[0] as i32;
                let dy = cur_node[1] as i32 - parent_pos[1] as i32;
                let direction = if dx == 0 {
                    if dy >= 1 {Dir::Down}
                    else if dy <= -1 {Dir::Up}
                    else {Dir::None}
                } else if dx >= 1 {
                    if dy >= 1 {Dir::RightDown}
                    else if dy == 0 {Dir::Right}
                    else if dy <= -1 {Dir::RightUp}
                    else {Dir::None}
                } else if dx <= -1 {
                    if dy >= 1 {Dir::LeftDown}
                    else if dy == 0 {Dir::Left}
                    else if dy <= -1 {Dir::LeftUp}
                    else {Dir::None}
                } else {Dir::None};
                let mut natural = Vec::new();
                let mut forced = Vec::new();
                match direction {
                    Dir::Right => {
                        if(!self.map.get(&[cur_node[0] + 1,cur_node[1]]).unwrap_or(&true)) {
                            natural.push([cur_node[0] + 1,cur_node[1]]);
                        }
                        if(*self.map.get(&[cur_node[0],cur_node[1]-1]).unwrap_or(&false)) {
                            forced.push([cur_node[0]+1,cur_node[1]-1]);
                        }
                        if(*self.map.get(&[cur_node[0],cur_node[1]+1]).unwrap_or(&false)) {
                            forced.push([cur_node[0]+1,cur_node[1]+1]);
                        }
                    }
                    Dir::Left => {
                        if(!self.map.get(&[cur_node[0] - 1,cur_node[1]]).unwrap_or(&true)) {
                            natural.push([cur_node[0] - 1,cur_node[1]]);
                        }
                        if(*self.map.get(&[cur_node[0],cur_node[1]-1]).unwrap_or(&false)) {
                            forced.push([cur_node[0]-1,cur_node[1]-1]);
                        }
                        if(*self.map.get(&[cur_node[0],cur_node[1]+1]).unwrap_or(&false)) {
                            forced.push([cur_node[0]-1,cur_node[1]+1]);
                        }
                    }
                    Dir::Up => {
                        if(!self.map.get(&[cur_node[0],cur_node[1] - 1]).unwrap_or(&true)) {
                            natural.push([cur_node[0],cur_node[1] - 1]);
                        }
                        if(*self.map.get(&[cur_node[0] - 1,cur_node[1]]).unwrap_or(&false)) {
                            forced.push([cur_node[0]-1,cur_node[1]-1]);
                        }
                        if(*self.map.get(&[cur_node[0] + 1,cur_node[1]]).unwrap_or(&false)) {
                            forced.push([cur_node[0]+1,cur_node[1]-1]);
                        }
                    }
                    Dir::Down => {
                        if(!self.map.get(&[cur_node[0],cur_node[1] + 1]).unwrap_or(&true)) {
                            natural.push([cur_node[0],cur_node[1] + 1]);
                        }
                        if(*self.map.get(&[cur_node[0] - 1,cur_node[1]]).unwrap_or(&false)) {
                            forced.push([cur_node[0]-1,cur_node[1]+1]);
                        }
                        if(*self.map.get(&[cur_node[0] + 1,cur_node[1]]).unwrap_or(&false)) {
                            forced.push([cur_node[0]+1,cur_node[1]+1]);
                        }
                    }
                    Dir::RightUp => {
                        if(!self.map.get(&[cur_node[0],cur_node[1] - 1]).unwrap_or(&true)) {
                            natural.push([cur_node[0],cur_node[1] - 1]);
                        }
                        if(!self.map.get(&[cur_node[0] + 1,cur_node[1]]).unwrap_or(&true)) {
                            natural.push([cur_node[0] + 1,cur_node[1]]);
                        }
                        if(!self.map.get(&[cur_node[0] + 1,cur_node[1] - 1]).unwrap_or(&true)) {
                            natural.push([cur_node[0] + 1,cur_node[1] - 1]);
                        }
                        if(*self.map.get(&[cur_node[0],cur_node[1] + 1]).unwrap_or(&false)) {
                            forced.push([cur_node[0]+1,cur_node[1]+1]);
                        }
                        if(*self.map.get(&[cur_node[0] - 1,cur_node[1]]).unwrap_or(&false)) {
                            forced.push([cur_node[0]-1,cur_node[1]-1]);
                        }
                    }
                    Dir::RightDown => {
                        if(!self.map.get(&[cur_node[0],cur_node[1] + 1]).unwrap_or(&true)) {
                            natural.push([cur_node[0],cur_node[1] + 1]);
                        }
                        if(!self.map.get(&[cur_node[0] + 1,cur_node[1]]).unwrap_or(&true)) {
                            natural.push([cur_node[0] + 1,cur_node[1]]);
                        }
                        if(!self.map.get(&[cur_node[0] + 1,cur_node[1] + 1]).unwrap_or(&true)) {
                            natural.push([cur_node[0] + 1,cur_node[1] + 1]);
                        }
                        if(*self.map.get(&[cur_node[0],cur_node[1] - 1]).unwrap_or(&false)) {
                            forced.push([cur_node[0]+1,cur_node[1]-1]);
                        }
                        if(*self.map.get(&[cur_node[0] - 1,cur_node[1]]).unwrap_or(&false)) {
                            forced.push([cur_node[0]-1,cur_node[1]+1]);
                        }
                    }
                    Dir::LeftUp => {
                        if(!self.map.get(&[cur_node[0],cur_node[1] - 1]).unwrap_or(&true)) {
                            natural.push([cur_node[0],cur_node[1] - 1]);
                        }
                        if(!self.map.get(&[cur_node[0] - 1,cur_node[1]]).unwrap_or(&true)) {
                            natural.push([cur_node[0] - 1,cur_node[1]]);
                        }
                        if(!self.map.get(&[cur_node[0] - 1,cur_node[1] - 1]).unwrap_or(&true)) {
                            natural.push([cur_node[0] - 1,cur_node[1] - 1]);
                        }
                        if(*self.map.get(&[cur_node[0],cur_node[1] + 1]).unwrap_or(&false)) {
                            forced.push([cur_node[0]-1,cur_node[1]+1]);
                        }
                        if(*self.map.get(&[cur_node[0] + 1,cur_node[1]]).unwrap_or(&false)) {
                            forced.push([cur_node[0]+1,cur_node[1]-1]);
                        }
                    }
                    Dir::LeftDown => {
                        if(!self.map.get(&[cur_node[0],cur_node[1] + 1]).unwrap_or(&true)) {
                            natural.push([cur_node[0],cur_node[1] + 1]);
                        }
                        if(!self.map.get(&[cur_node[0] - 1,cur_node[1]]).unwrap_or(&true)) {
                            natural.push([cur_node[0] - 1,cur_node[1]]);
                        }
                        if(!self.map.get(&[cur_node[0] - 1,cur_node[1] + 1]).unwrap_or(&true)) {
                            natural.push([cur_node[0] - 1,cur_node[1] + 1]);
                        }
                        if(*self.map.get(&[cur_node[0],cur_node[1] - 1]).unwrap_or(&false)) {
                            forced.push([cur_node[0]-1,cur_node[1]-1]);
                        }
                        if(*self.map.get(&[cur_node[0] + 1,cur_node[1]]).unwrap_or(&false)) {
                            forced.push([cur_node[0]+1,cur_node[1]+1]);
                        }
                    }
                    Dir::None => {
                        panic!("ERROR");
                    }

                }


            return (natural, forced);
            }




    }


    fn successors(&self, cur_node: [usize; 2], start: [usize; 2], goal: [usize; 2]) -> Vec<([usize;2],f32)>{
        let mut successors = Vec::new();
        let mut n = self.prune(cur_node, self.nodes.get(&cur_node).and_then(|x| x.parent));
        let mut neighbours = n.0;
        neighbours.extend(n.1);

        let cn = self.nodes.get(&cur_node).map(|x| x.cost).unwrap_or(0.);
        for neighbour in neighbours.iter_mut() {
            if let Some(nei) = self.jump(cur_node, [neighbour[0] as i32 - cur_node[0] as i32, neighbour[1] as i32 - cur_node[1] as i32],start,goal,cn) {
                successors.push(nei);
            }
        }

        successors

    }
    fn jump(&self, cur_node: [usize;2], direction: [i32;2], start: [usize;2], goal:[usize;2], cost: f32) -> Option<([usize;2], f32)> {
        let diagonal = if (direction[0].abs() == 1 && direction[1].abs() == 1) {true}else{false};
        let inc_cost = cost + if diagonal{(2_f32).sqrt()}else{1.0};
        let nx = cur_node[0] as i32 + direction[0] as i32;
        if nx < 0 {return None}
        let ny = cur_node[1] as i32 + direction[1] as i32;
        if ny < 0 {return None}
        let n = [nx as usize, ny as usize];
        if *self.map.get(&n).unwrap_or(&true) {return None;}
        if n == goal {return Some((n,inc_cost));}
        if self.prune(n, Some(cur_node)).1.len() > 0 {return Some((n,inc_cost));}
        if diagonal {
            if (self.jump(n,[direction[0],0],start,goal,cost + 1.0).is_some()) {return Some((n,inc_cost));}
            if (self.jump(n,[0,direction[1]],start,goal, cost+1.0).is_some()) {return Some((n,inc_cost));}
        }
        return self.jump(n,direction,start,goal,inc_cost);
    }

    fn find(&mut self, start: [usize;2], goal: [usize;2]) -> (Vec<Dir>, f32) {
        let mut open_nodes: BTreeSet<Node> = BTreeSet::new();
        let mut closed_nodes = HashSet::new();
        open_nodes.insert(Node::new(None,0.0,start,goal));
        self.nodes.insert(start,Node::new(None,0.0,start,goal));

        while let Some(cur) = open_nodes.pop_first() {
            if cur.position == goal {
                return self.path_from(cur);
            }

            let neighbours = self.successors(cur.position, start, goal); 

            for neighbour in neighbours.iter() {
                if (!self.map.contains_key(&neighbour.0)){continue;}
                if !closed_nodes.contains(&neighbour.0) && !self.map.get(&neighbour.0).unwrap() {
                    if open_nodes.iter().any(|x| x.position == cur.position) {
                        let mut cn = open_nodes.iter().find(|x| x.position == cur.position).unwrap().clone();
                        if (cur.cost + neighbour.1) < cn.cost {
                            cn.parent = Some(cur.position);
                            cn.cost = cur.cost + neighbour.1;
                            cn.f = cn.cost + cn.hueristic_dist * 1.0001;
                        }
                        open_nodes.insert(cn.clone());
                        self.nodes.insert(cn.position,cn);
                    }else {
                        self.nodes.insert(neighbour.0, Node::new(Some(cur.position), cur.cost + neighbour.1, neighbour.0, goal));
                        open_nodes.insert(Node::new(Some(cur.position), cur.cost + neighbour.1, neighbour.0, goal));
                    }
                }
            }
            closed_nodes.insert(cur.position);
        }
        return (vec![],0.);

    }

    fn path_from(&self, end_node: Node) -> (Vec<Dir>, f32) {
        let mut dir = Vec::new();
        let mut n = &end_node;
        let mut cost = 0.;
        while let Some(p) = n.parent {
            let par = self.nodes.get(&p).unwrap();
            let np = n.position;
            let pp = par.position;
            n = par;
            let dx = np[0] as i32 -pp[0] as i32; // 1 = right 
            let dy = np[1] as i32 - pp[1] as i32;


            if dy.abs() > 1 && dx.abs() > 1 && dx.abs() == dy.abs(){
                if dx > 1 && dy > 1{
                    for i in 0..dx {
                        dir.push(Dir::RightDown);
                        cost += (2.0_f32).sqrt();
                    }
                }
                if dx > 1 && dy < -1{
                    for i in 0..dx {
                        dir.push(Dir::RightUp);
                        cost += (2.0_f32).sqrt();
                    }
                }
                if dx < -1 && dy > 1{
                    for i in 0..dy {
                        dir.push(Dir::LeftDown);
                        cost += (2.0_f32).sqrt();
                    }
                }
                if dx < -1 && dy < -1{
                    for i in 0..dy.abs() {
                        dir.push(Dir::LeftUp);
                        cost += (2.0_f32).sqrt();
                    }
                }
            } else if dy == 0 && dx.abs() > 1 {
                if dx > 1 {
                    for i in 0..dx.abs() {
                        dir.push(Dir::Right);
                        cost += 1.; 
                    }
                } else {
                    for i in 0..dx.abs() {
                        dir.push(Dir::Left);
                        cost += 1.; 
                    }
                }
            } else if dx == 0 && dy.abs() > 1 {
                if dy > 1 {
                    for i in 0..dy {
                        dir.push(Dir::Down); 
                        cost += 1.; 
                    }
                } else {
                    for i in 0..dy.abs() {
                        dir.push(Dir::Up);
                        cost += 1.; 
                    }
                }
            }

            else {
                if (dx.abs() == 1 && dy.abs() == 1) {
                    cost += (2.0_f32).sqrt();
                } else {
                    cost += 1.;
                }
                dir.push(
                    if dx == -1 {
                        if dy == -1 { Dir::LeftUp }
                        else if dy == 0 { Dir::Left }
                        else if dy == 1 { Dir::LeftDown }
                        else { Dir::None }
                    } else if dx == 0 {
                        if dy == -1 { Dir::Up }
                        else if dy == 1 { Dir::Down }
                        else { Dir::None }
                    } else if dx == 1 {
                        if dy == -1 { Dir::RightUp }
                        else if dy == 0 { Dir::Right }
                        else if dy == 1 { Dir::RightDown }
                        else { Dir::None }
                    } 
                    else { Dir::None } 
                );
            }

        }
        return (dir, cost);
    }
    /************************************************************************************************************************************************************************
     **************************************************************************** Dijkstra's *******************************************************************************
    ************************************************************************************************************************************************************************/

    fn path_from_d(&self, end_node: NodeD) -> (Vec<Dir>, f32) {
        let mut cost = 0.;
        let mut dir = Vec::new();
        let mut n = &end_node;
        while let Some(p) = n.parent {
            let par = self.nodes_d.get(&p).unwrap();
            let np = n.position;
            let pp = par.position;
            n = par;
            let dx = np[0] as i32 -pp[0] as i32; // 1 = right 
            let dy = np[1] as i32 - pp[1] as i32;

            if dy.abs() > 1 && dx.abs() > 1 && dx.abs() == dy.abs(){
                if dx > 1 && dy > 1{
                    for i in 0..dx {
                        dir.push(Dir::RightDown);
                        cost += (2.0_f32).sqrt();
                    }
                }
                if dx > 1 && dy < -1{
                    for i in 0..dx {
                        dir.push(Dir::RightUp);
                        cost += (2.0_f32).sqrt();
                    }
                }
                if dx < -1 && dy > 1{
                    for i in 0..dy {
                        dir.push(Dir::LeftDown);
                        cost += (2.0_f32).sqrt();
                    }
                }
                if dx < -1 && dy < -1{
                    for i in 0..dy.abs() {
                        dir.push(Dir::LeftUp);
                        cost += (2.0_f32).sqrt();
                    }
                }
            } else if dy == 0 && dx.abs() > 1 {
                if dx > 1 {
                    for i in 0..dx.abs() {
                        dir.push(Dir::Right);
                        cost += 1.; 
                    }
                } else {
                    for i in 0..dx.abs() {
                        dir.push(Dir::Left);
                        cost += 1.; 
                    }
                }
            } else if dx == 0 && dy.abs() > 1 {
                if dy > 1 {
                    for i in 0..dy {
                        dir.push(Dir::Down); 
                        cost += 1.; 
                    }
                } else {
                    for i in 0..dy.abs() {
                        dir.push(Dir::Up);
                        cost += 1.; 
                    }
                }
            }

            else {
                if (dx.abs() == 1 && dy.abs() == 1) {
                    cost += (2.0_f32).sqrt();
                } else {
                    cost += 1.;
                }
                dir.push(
                    if dx == -1 {
                        if dy == -1 { Dir::LeftUp }
                        else if dy == 0 { Dir::Left }
                        else if dy == 1 { Dir::LeftDown }
                        else { Dir::None }
                    } else if dx == 0 {
                        if dy == -1 { Dir::Up }
                        else if dy == 1 { Dir::Down }
                        else { Dir::None }
                    } else if dx == 1 {
                        if dy == -1 { Dir::RightUp }
                        else if dy == 0 { Dir::Right }
                        else if dy == 1 { Dir::RightDown }
                        else { Dir::None }
                    } 
                    else { Dir::None } 


                );
            }

        }
        return (dir,cost);
    }


    fn Dijkstra(&mut self, start: [usize;2], goal:[usize;2]) -> (Vec<Dir>, f32){
        let mut open_nodes: BTreeSet<NodeD> = BTreeSet::new();
        let mut closed_nodes = HashSet::new();
        open_nodes.insert(NodeD::new(None,0.0,start));
        self.nodes_d.insert(start,NodeD::new(None,0.0,start));

        while let Some(cur) = open_nodes.pop_first() {
            if cur.position == goal {
                return self.path_from_d(cur);
            }

            let cp = cur.position;
            let neighbours =[
                ([cp[0] + 1, cp[1]], 1.),
                ([cp[0] - 1, cp[1]], 1.),
                ([cp[0] + 1, cp[1] + 1], (2.0_f32).sqrt()),
                ([cp[0] - 1, cp[1] + 1], (2.0_f32).sqrt()),
                ([cp[0] + 1, cp[1] - 1], (2.0_f32).sqrt()),
                ([cp[0] - 1, cp[1] - 1], (2.0_f32).sqrt()),
                ([cp[0], cp[1] - 1], 1.),
                ([cp[0], cp[1] + 1], 1.),
            ] ; 

            for neighbour in neighbours.iter() {
                if (!self.map.contains_key(&neighbour.0)){continue;}
                if !closed_nodes.contains(&neighbour.0) && !self.map.get(&neighbour.0).unwrap() {
                    if open_nodes.iter().any(|x| x.position == neighbour.0) {
                        let mut cn = open_nodes.iter().find(|x| x.position == neighbour.0).unwrap().clone();
                        if (cur.cost + neighbour.1) < cn.cost {
                            cn.parent = Some(cur.position);
                            cn.cost = cur.cost + neighbour.1;
                        }
                        open_nodes.insert(cn.clone());
                        self.nodes_d.insert(cn.position,cn);
                    }else {
                        self.nodes_d.insert(neighbour.0, NodeD::new(Some(cur.position), cur.cost + neighbour.1, neighbour.0));
                        open_nodes.insert(NodeD::new(Some(cur.position), cur.cost + neighbour.1, neighbour.0));
                    }
                }
            }
            closed_nodes.insert(cur.position);
        }
        return (vec![],0.);
    }




}



fn main() {
    let mut rng = rand::rng();
    for size in 1..=25 {
        
        let mut nanos_j: f64 = 0.;
        let mut nanos_d: f64 = 0.;
        let mut nanos_a: f64 = 0.;
        for iteration in 0..50 {
            let mut hash = HashMap::new();
            for x in 0..size*20 {
                for y in 0..size*20 {
                    hash.insert([x,y],false);
                }
            }
            let mut map = Map::new(hash);
            let time = Instant::now();
            let JPS = map.find([0,0],[size*20 - 1,size * 20 - 1]);
            nanos_j += time.elapsed().as_nanos() as f64;



            let time = Instant::now();
            let DIJ = map.Dijkstra([0,0],[size * 20 - 1,size * 20 - 1]);
            nanos_d += time.elapsed().as_nanos() as f64;


            // assert equal costs
            assert_eq!(JPS.1, DIJ.1);
        }
        println!("JPS average over 50 iterations for size {}: {:?} ns", size*20, nanos_j/50.);
        println!("DIJ average over 50 iterations for size {}: {:?} ns", size*20, nanos_d/50.);
    }

}


