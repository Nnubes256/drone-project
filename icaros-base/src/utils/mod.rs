//! Utility structures and functions

pub mod crc16;
pub mod quartenion;
pub mod vector;

pub use quartenion::Quartenion;
pub use vector::Point3;

use std::convert::TryInto;

/// Returns true if all the elements on a given array are the same
///
/// Source: https://mastodon.technology/@bugaevc/102226891784062955
pub fn is_all_same<T: PartialEq>(arr: &[T]) -> bool {
    arr.windows(2).all(|w| w[0] == w[1])
}

/// Reads an unsigned 2-byte number out of a byte array
///
/// Source: https://doc.rust-lang.org/std/primitive.u16.html#method.from_le_bytes
pub fn read_le_u16(input: &[u8]) -> u16 {
    let (int_bytes, _rest) = input.split_at(std::mem::size_of::<u16>());
    u16::from_le_bytes(int_bytes.try_into().unwrap())
}

/// Maps a given value X from a given input range to the given output range
///
/// Source: https://www.arduino.cc/reference/en/language/functions/math/map/
pub fn map_values(x: f64, in_range: (f64, f64), out_range: (f64, f64)) -> f64 {
    (x - in_range.0) * (out_range.1 - out_range.0) / (in_range.1 - in_range.0) + out_range.0
}

/*pub fn map_values<A, B>(x: A, in_range: (A, A), out_range: (B, B)) -> B
    where A: Copy + Sub<Output = A> + Mul<Output = A>,
          B: Copy + Add<Output = B> + Sub<Output = B> + Mul<Output = B> + Div<Output = B> + From<A>
{
    let temp0: B = (x - in_range.0).into();
    temp0 * (out_range.1 - out_range.0) / (in_range.1 - in_range.0).into() + out_range.0
}*/

/// Maps a signed 1-byte value into an unsigned 1-byte value
pub fn map_i8_into_u8(x: i8) -> u8 {
    let (x, _) = (x as u8).overflowing_sub(128);
    x
}
