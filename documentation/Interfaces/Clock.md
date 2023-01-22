Our clock is an RTC that lets us determine times. While most of it is handled by the [[HAL]], we have several internal functions for convenience, like `datetime_difference`, which measures the difference between two dates, in seconds.

```rust
//func-def datetime_difference
fn datetime_difference(start: &hal::rtc::DateTime, end: &hal::rtc::DateTime) -> f32 {
	let mut seconds = 0.0;
	seconds += (end.year - start.year) as f32 * 365.0 * 24.0 * 60.0 * 60.0;
	seconds += (end.month - start.month) as f32 * 30.0 * 24.0 * 60.0 * 60.0;
	seconds += (end.day - start.day) as f32 * 24.0 * 60.0 * 60.0;
	seconds += (end.hour - start.hour) as f32 * 60.0 * 60.0;
	seconds += (end.minute - start.minute) as f32 * 60.0;
	seconds += (end.second - start.second) as f32;
	seconds 
}
```