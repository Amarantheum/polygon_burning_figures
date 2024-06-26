use std::collections::HashMap;
use std::fs::File;

use image::{ColorType, ImageBuffer, Rgb};
use geo::{euclidean_distance, Contains, Coord, EuclideanLength, Line, LineString, Point, Polygon};
use geo::algorithm::intersects::Intersects;
use geo::algorithm::line_intersection::line_intersection;
use plotters::backend::RGBPixel;
use plotters::coord::Shift;
use plotters::prelude::*;
use itertools::Itertools;
use serde::{Deserialize, Serialize};

fn main() {
    let vertices = vec![(0, 3), (1, 6), (2, 4), (3, 5), (4, 5), (5, 6), (6, 3), (8, 5), (8, 4), (7, 2), (7, 0), (5, 1), (4, 1), (2, 2), (1, 1)];
    let polygon = SimplePolygon::new(vertices);
    let scale = 100;

    // let file = match File::open("dist_table.json") {
    //     Ok(file) => file,
    //     Err(_) => File::create("dist_table.json").unwrap(),
    // };
    // let dist_table: Vec<((u32, u32), Vec<(usize, f64)>)> = match serde_json::from_reader(file) {
    //     Ok(dist_table) => dist_table,
    //     Err(_) => vec![],
    // };
    let dist_table = HashMap::new();
    //let dist_table: DistTable = dist_table.into_iter().map(|(key, values)| (key, values.into_iter().collect())).collect();

    let dist_table = polygon.draw("simple_polygon.png", (8 * scale, 6 * scale), scale as f64, 2, dist_table).unwrap();



    // let file = File::create("dist_table.json").unwrap();
    // let dist_table: Vec<((u32, u32), Vec<(usize, f64)>)> = dist_table.into_iter().map(|(key, values)| (key, values.into_iter().collect())).collect();
    // serde_json::to_writer(file, &dist_table).unwrap();
}

type DistTable = HashMap<(u32, u32), HashMap<usize, f64>>;

pub struct SimplePolygon {
    polygon: Polygon<f64>,
    visibility_graph: Vec<Vec<bool>>,
    shortest_paths: Vec<Vec<Option<LineString>>>,
}

impl SimplePolygon {
    pub fn new(vertices: Vec<(usize, usize)>) -> Self {
        let coordinates: Vec<_> = vertices.iter().map(|(x, y)| (*x as f64, *y as f64)).collect();
        let line_string = LineString::from(coordinates.clone());
        let polygon = Polygon::new(line_string.clone(), vec![]);
        let visibility_graph = SimplePolygon::generate_visibility_graph(&polygon);
        let shortest_paths = SimplePolygon::generate_shortest_paths(&polygon, &visibility_graph);
        SimplePolygon {
            polygon,
            visibility_graph,
            shortest_paths,
        }
    }

    fn proper_intersection(line1: &Line<f64>, line2: &Line<f64>) -> bool {
        let intersection = line_intersection(line1.clone(), line2.clone());
        if let Some(intersection) = intersection {
            if intersection.is_proper() {
                return true;
            }
        }
        false
    }

    fn polygon_contains(polygon: &Polygon<f64>, line: &Line<f64>) -> bool {
        if polygon.contains(line) {
            return true;
        }

        for edge in polygon.exterior().lines() {
            if edge == *line || edge == Line::new(line.end, line.start) {
                return true;
            }
        }

        false
    }

    pub fn generate_visibility_graph(polygon: &Polygon<f64>) -> Vec<Vec<bool>> {
        let mut visibility_graph = vec![vec![false; polygon.exterior().points().count()]; polygon.exterior().points().count()];
        
        for (i, vertex) in polygon.exterior().points().enumerate() {
            for (j, other_vertex) in polygon.exterior().points().enumerate() {
                if i != j {
                    let line = Line::new(vertex, other_vertex);
                    visibility_graph[i][j] = Self::polygon_contains(polygon, &line);
                }
            }
        }
        visibility_graph
    }

    pub fn generate_shortest_paths(polygon: &Polygon<f64>, visibility_graph: &Vec<Vec<bool>>) -> Vec<Vec<Option<LineString>>> {
        let mut shortest_paths: Vec<Vec<Option<LineString>>> = vec![vec![None; polygon.exterior().points().count()]; polygon.exterior().points().count()];

        let mut visible_vertices: Vec<Vec<usize>> = vec![vec![]; polygon.exterior().points().count()];

        for (i, vertex) in polygon.exterior().points().enumerate() {
            for (j, other_vertex) in polygon.exterior().points().enumerate() {
                if visibility_graph[i][j] {
                    visible_vertices[i].push(j);
                    shortest_paths[i][j] = Some(LineString::from(vec![vertex, other_vertex]));
                }
            }
        }

        let mut update = true;
        let points = polygon.exterior().points().collect::<Vec<_>>();

        while update {
            update = false;

            for i in 0..points.len() {
                for j in 0..points.len() {
                    if i == j {
                        continue;
                    }

                    let best_path = &shortest_paths[i][j];
                    let best_distance = if let Some(path) = best_path.as_ref() {
                        path.euclidean_length()
                    } else {
                        f64::INFINITY
                    };
                    for k in &visible_vertices[i] {
                        if let Some(path) = shortest_paths[*k][j].as_ref() {
                            let distance = path.euclidean_length() + shortest_paths[i][*k].as_ref().unwrap().euclidean_length();
                            if (distance * 10000.0).round() < (best_distance * 10000.0).round() {
                                let mut new_points = vec![points[i]];
                                new_points.extend(shortest_paths[*k][j].as_ref().unwrap().points());
                                shortest_paths[i][j] = Some(LineString::from(new_points));
                                assert_eq!((shortest_paths[i][j].as_ref().unwrap().euclidean_length() * 10000.0).round(), (distance * 10000.0).round());
                                update = true;
                            }
                        }
                    }
                }
            }
        }

        shortest_paths
    }

    pub fn shortest_path(&self, start: (f64, f64), end: usize) -> Option<LineString> {
        if self.polygon.contains(&Point(Coord { x: start.0, y: start.1 })) {
            let mut best_shortest_path = None;
            let mut best_distance = f64::INFINITY;
            for (i, vertex) in self.polygon.exterior().points().enumerate() {
                if Self::polygon_contains(&self.polygon, &Line::new(Point(Coord { x: start.0, y: start.1 }), vertex)) {
                    if i == end {
                        return Some(LineString::from(vec![Point(Coord { x: start.0, y: start.1 }), vertex]));
                    }
                    let mut shortest_path = vec![Point(Coord { x: start.0, y: start.1 })];
                    shortest_path.extend(self.shortest_paths[i][end].as_ref().unwrap().points());
                    let shortest_path = LineString::from(shortest_path);
                    let distance = shortest_path.euclidean_length();
                    if distance < best_distance {
                        best_distance = distance;
                        best_shortest_path = Some(shortest_path);
                    }
                }
            }
            best_shortest_path
        } else {
            None
        }
    }

    pub fn contains(&self, point: (f64, f64)) -> bool {
        self.polygon.contains(&Point(Coord { x: point.0, y: point.1 }))
    }

    pub fn create_image(&self, width: u32, height: u32) -> ImageBuffer<Rgb<u8>, Vec<u8>> {
        let mut img = ImageBuffer::new(width, height);

        for (x, y, pixel) in img.enumerate_pixels_mut() {
            let inside = self.contains((x as f64 / 100.0, (-(y as f64) + height as f64) / 100.0));
            *pixel = if inside {
                image::Rgb([255, 0, 0]) // Red for inside
            } else {
                image::Rgb([255, 255, 255]) // White for outside
            };
        }

        img
    }

    pub fn draw(&self, filename: &str, size: (u32, u32), scale: f64, k: usize, dist_table: DistTable) -> Result<DistTable, Box<dyn std::error::Error>> {
        let (width, height) = size;  // adjust as needed
        let root = BitMapBackend::new(filename, (width, height)).into_drawing_area();
        root.fill(&WHITE)?;

        let mut chart = ChartBuilder::on(&root)
            .margin(0)
            .set_all_label_area_size(0)
            .build_cartesian_2d(0f64..width as f64, 0f64..height as f64)?;

        chart.configure_mesh().x_labels(0).y_labels(0).draw()?;
        
        //let (best_combination, dist_table) = self.get_optimal_solution(size, scale, k, dist_table)?;
        //let dist_table = self.draw_voronoi_regions(&root, size, scale, best_combination, dist_table, true)?;
        let dist_table = self.draw_voronoi_regions(&root, size, scale, vec![1,14,7], dist_table, true)?;
        Ok(dist_table)
    }


    fn get_optimal_solution(&self, size: (u32, u32), scale: f64, k: usize, mut dist_table: DistTable) -> Result<(Vec<usize>, DistTable), Box<dyn std::error::Error>> {
        let (width, height) = size;
        let all_ks: Vec<Vec<usize>> = (0..self.polygon.exterior().points().count() - 1)
            .combinations(k)
            .collect();
        let len = all_ks.len();

        let mut best_combination = vec![];
        let mut best_final_burn_distance = f64::INFINITY;
        let mut count = 0;

        for combination in all_ks {
            println!("Combination {} of {}", count, len);
            count += 1;
            let mut final_burn_point = (0, 0);
            let mut final_burn_distance = -f64::INFINITY;
            // Draw the polygon
            for x in 0..width {
                for y in 0..height {
                    let x_scaled = x as f64 / scale;
                    let y_scaled = (height as f64 - y as f64) / scale;
                    let point = Point::new(x_scaled, y_scaled);
                    
                    if self.polygon.contains(&point) {
                        let distances: Vec<(usize, f64)> = combination.iter().map(|i| {
                            let d = *dist_table.entry((x, y)).or_insert(HashMap::new()).entry(*i).or_insert_with(|| self.shortest_path((x_scaled, y_scaled), *i).unwrap().euclidean_length());
                            (*i, d)
                        }).collect();
                        let (i, d) = distances.iter().min_by(|a, b| a.1.partial_cmp(&b.1).unwrap()).unwrap();
                        if *d > final_burn_distance {
                            final_burn_distance = *d;
                            final_burn_point = (x, y);
                        }
                    }
                }
            }
            //root.draw_pixel((final_burn_point.0 as i32, final_burn_point.1 as i32), &BLACK)?;
            if final_burn_distance < best_final_burn_distance {
                println!("New best final burn distance: {}, Combination: {:?}", final_burn_distance, combination);
                best_final_burn_distance = final_burn_distance;
                best_combination = combination;
            }
        }
        println!("{:?}", best_combination);
        println!("{}", best_final_burn_distance);
        Ok((best_combination, dist_table))
    }


    fn draw_voronoi_regions(&self, root: &DrawingArea<BitMapBackend<RGBPixel>, Shift>, img_size: (u32, u32), scale: f64, combination: Vec<usize>, mut dist_table: DistTable, add_fbp: bool) -> Result<DistTable, Box<dyn std::error::Error>> {
        let (width, height) = img_size;
        let k = combination.len();

        let mut final_burn_point = (0, 0);
        let mut final_burn_distance = -f64::INFINITY;

        let colors: HashMap<usize, RGBColor> = combination.iter().enumerate().map(|(i, vertex)| (*vertex, RGBColor((255.0 * i as f64 / k as f64) as u8, 0, 0))).collect();

        for x in 0..width {
            for y in 0..height {
                let x_scaled = x as f64 / scale;
                let y_scaled = (height as f64 - y as f64) / scale;
                let point = Point::new(x_scaled, y_scaled);
                
                if self.polygon.contains(&point) {
                    let distances: Vec<(usize, f64)> = combination.iter().map(|i| {
                        let d = *dist_table.entry((x, y)).or_insert(HashMap::new()).entry(*i).or_insert_with(|| self.shortest_path((x_scaled, y_scaled), *i).unwrap().euclidean_length());
                        (*i, d)
                    }).collect();
                    let (i, d) = distances.iter().min_by(|a, b| a.1.partial_cmp(&b.1).unwrap()).unwrap();
                    if *d > final_burn_distance {
                        final_burn_distance = *d;
                        final_burn_point = (x, y);
                    }
                    root.draw_pixel((x as i32, y as i32), &colors[i])?;
                }
            }
        }
        if add_fbp {
            root.draw_pixel((final_burn_point.0 as i32, final_burn_point.1 as i32), &GREEN)?;
        }
        Ok(dist_table)
    }
}

