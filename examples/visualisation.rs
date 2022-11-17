use std::io::Read;
use plotters::prelude::*;
use ukalman::Kalman1D;

fn main() {
    let mut f = std::fs::File::open("test_assets/data.txt").unwrap();
    let mut str = String::new();
    f.read_to_string(&mut str).unwrap();
    let data = str.split(", ").flat_map(|s| s.parse::<u16>());

    let root_drawing_area = BitMapBackend::new("images/2.4.png", (600, 400))
        .into_drawing_area();

    root_drawing_area.fill(&WHITE).unwrap();

    let mut ctx = ChartBuilder::on(&root_drawing_area)
        .caption("Filtering noisy signal", ("Arial", 30))
        .set_label_area_size(LabelAreaPosition::Left, 40)
        .set_label_area_size(LabelAreaPosition::Bottom, 40)
        .build_cartesian_2d(0..10000isize, 0..3000isize)
        .unwrap();

    ctx.configure_mesh().draw().unwrap();

    ctx.draw_series(LineSeries::new(data.scan(0isize, |x, y| {
        *x += 32;
        Some((*x, y as isize))
    }), &BLUE)).unwrap()
        .label("Raw signal")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));

    let mut kal = Kalman1D::new(0.0, 10.0);

    ctx.draw_series(LineSeries::new(str.split(", ").flat_map(|s| s.parse::<u16>()).scan(0isize, |x, y| {
        *x += 32;
        Some((*x, kal.filter(y as f32, 5.0, |x| x, |s| s + 0.0001) as isize))
    }), &RED)).unwrap()
        .label("Kalman filtered")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

    ctx.configure_series_labels()
        .border_style(&BLACK)
        .background_style(&WHITE.mix(0.8))
        .draw()
        .unwrap();
}