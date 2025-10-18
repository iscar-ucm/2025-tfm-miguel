use plotters::{coord::{types::{RangedCoordf64}, Shift}, prelude::*};

use crate::discrete_time_model::{transducer::Transducer, types::{Quaternion, Vec3}};

const OUT_FILE_NAME: &str = "images/Simulation Result.png";

pub fn draw(transducer: &Transducer, total_time: f64) {
    let root = BitMapBackend::new(OUT_FILE_NAME, (1600, 1200))
        .into_drawing_area();
    root.fill(&WHITE).unwrap();

    let areas = root.split_evenly((3, 1));
    let ten_percent = total_time * 0.1;
    let max_x = total_time + ten_percent;
    let dt = total_time / transducer.get_q_error_history().len() as f64;

    draw_q_error(&areas[0], transducer.get_q_error_history(), ten_percent, max_x, dt);
    draw_w_history(&areas[1], transducer.get_w_history(), ten_percent, max_x, dt);
    draw_rw_speeds_history(&areas[2], transducer.get_rw_speeds_history(), ten_percent, max_x, dt);

    root.present().expect("Unable to write result to file, please make sure 'images' dir exists under current dir");
    println!("Nanosatellite Attitude Control Simulation has been saved to {}", OUT_FILE_NAME);
}

fn draw_q_error(area: &DrawingArea<BitMapBackend, Shift>, data: Vec<Quaternion>, min_x: f64, max_x: f64, dt: f64) {
    let mut ctx = ChartBuilder::on(area)
    .set_label_area_size(LabelAreaPosition::Left, 60)
    .set_label_area_size(LabelAreaPosition::Bottom, 40)
    .margin(30)
    .caption("Nanosatellite Attitude Control Simulation", ("sans-serif", 40))
    .build_cartesian_2d(-min_x..max_x, -0.02f64..0.25f64)
    .unwrap();

    configure_mesh(&mut ctx, "", "Attitude Error");

    let series_data: [(&RGBColor, &str, fn(&Quaternion) -> f64); 3] = [
        (&BLUE, "Error q_x", |q| q.0.i),
        (&RED, "Error q_y", |q| q.0.j),
        (&GREEN, "Error q_z", |q| q.0.k),
    ];

    for (color, label, extractor) in series_data {
        ctx.draw_series(LineSeries::new(
            data
                .iter()
                .enumerate()
                .map(|(i, q)| (i as f64 * dt, extractor(q))),
            ShapeStyle::from(color).stroke_width(2),
        )).unwrap()
        .label(label)
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], *color));
    }

    draw_series_labels(&mut ctx);
}

fn draw_w_history(area: &DrawingArea<BitMapBackend, Shift>, data: Vec<Vec3>, min_x: f64, max_x: f64, dt: f64) {
    let mut ctx = ChartBuilder::on(area)
        .margin(30)
        .set_label_area_size(LabelAreaPosition::Left, 60)
        .set_label_area_size(LabelAreaPosition::Bottom, 40)
        .build_cartesian_2d(-min_x..max_x, -0.12f64..0.25f64)
        .unwrap();

    configure_mesh(&mut ctx, "", "Angular Velocity [rad/s]");

    let series_data: [(&RGBColor, &str, fn(&Vec3) -> f64); 3] = [
        (&BLUE, "w_x", |v| v.0.x),
        (&RED, "w_y", |v| v.0.y),
        (&GREEN, "w_z", |v| v.0.z),
    ];

    for (color, label, extractor) in series_data {
        ctx.draw_series(LineSeries::new(
            data.iter().enumerate().map(|(i, v)| (i as f64 * dt, extractor(v))),
            ShapeStyle::from(color).stroke_width(2),
        )).unwrap()
        .label(label)
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], *color));
    }

    draw_series_labels(&mut ctx);
}

fn draw_rw_speeds_history(area: &DrawingArea<BitMapBackend, Shift>, data: Vec<Vec3>, min_x: f64, max_x: f64, dt: f64) {
    let mut ctx = ChartBuilder::on(area)
        .margin(30)
        .set_label_area_size(LabelAreaPosition::Left, 60)
        .set_label_area_size(LabelAreaPosition::Bottom, 40)
        .build_cartesian_2d(-min_x..max_x, -6.0f64..12.0f64)
        .unwrap();

    configure_mesh(&mut ctx, "Time [s]", "Reaction Wheel Speed [rad/s]");
    
    let series_data: [(&RGBColor, &str, fn(&Vec3) -> f64); 3] = [
        (&BLUE, "RW 1", |v| v.0.x),
        (&RED, "RW 2", |v| v.0.y),
        (&GREEN, "RW 3", |v| v.0.z),
    ];

    for (color, label, extractor) in series_data {
        ctx.draw_series(LineSeries::new(
            data.iter().enumerate().map(|(i, v)| (i as f64 * dt, extractor(v))),
            ShapeStyle::from(color).stroke_width(2),
        )).unwrap()
        .label(label)
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], *color));
    }

    draw_series_labels(&mut ctx);
}

fn draw_series_labels<'a>(ctx: &mut ChartContext<'a, BitMapBackend<'a>, Cartesian2d<RangedCoordf64, RangedCoordf64>>) {
    ctx.configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .position(SeriesLabelPosition::UpperRight)
        .draw()
        .unwrap();
}

fn configure_mesh<'a>(ctx: &mut ChartContext<'a, BitMapBackend<'a>, Cartesian2d<RangedCoordf64, RangedCoordf64>>, x: &str, y: &str) {
    ctx.configure_mesh()
        .x_desc(x)
        .y_desc(y)
        .label_style(("sans-serif", 20))
        .draw()
        .unwrap();
}