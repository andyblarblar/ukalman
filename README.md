# uKalman

Super tiny zero allocation filters for embedded.

Currently, this crate only implements a tiny 1D kalman filter, useful for smoothing ADCs and similar
1D sensors. In the future, multidimensional and non-linear filters may be added.

```rs
use std::io::Read;
use ukalman::Kalman1D;

// Initialises filter
let mut filter = Kalman1D::new(2000.0, 67.4f32.powi(2));

let mut f = std::fs::File::open("test_assets/cap_full_throttle.txt").unwrap();
let mut str = String::new();
f.read_to_string(&mut str).unwrap();

// Estimate over some data
for val in str.lines().flat_map(|s| s.parse::<u16>()) {
    // Filter with a static state model, and small system noise
    let est = filter.filter(val as f32, 191.93f32.powi(2), |x| x, |v| v + 0.001);
    std::println!("{:.0}", est);
}
```