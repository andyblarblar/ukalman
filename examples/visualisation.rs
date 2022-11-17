use std::io::Read;
use plotters::prelude::*;
use ukalman::Kalman1D;

fn main() {
    println!("Reading data...");
    let mut f = std::fs::File::open("test_assets/motor_on_progressive.txt").unwrap();
    let mut str = String::new();
    f.read_to_string(&mut str).unwrap();

    println!("Creating plot...");
    let root_drawing_area = BitMapBackend::new("/home/andy/CLionProjects/ukalman/2.4.png", (1000, 1000))
        .into_drawing_area();

    root_drawing_area.fill(&WHITE).unwrap();

    let mut ctx = ChartBuilder::on(&root_drawing_area)
        .caption("Filtering noisy signal", ("Arial", 30))
        .set_label_area_size(LabelAreaPosition::Left, 40)
        .set_label_area_size(LabelAreaPosition::Bottom, 40)
        .build_cartesian_2d(0..30000isize, 0..3000isize)
        .unwrap();

    ctx.configure_mesh().draw().unwrap();

    println!("starting drawing...");

    ctx.draw_series(LineSeries::new(str.lines().flat_map(|s| s.parse()).scan(0isize, |x, y: isize| {
        *x += 32;
        Some((*x, y as isize))
    }), &BLUE)).unwrap();

    println!("Done drawing normal...");

    let mut kal = Kalman1D::new(0.0, 10.0);

    ctx.draw_series(LineSeries::new(str.lines().flat_map(|s| s.parse()).scan(0isize, |x, y: isize| {
        *x += 32;
        Some((*x, kal.filter(y as f32, 0.006, |x| x, |s| s + 0.0001) as isize))
    }), &RED)).unwrap();

    println!("Done drawing kalman...");
}
