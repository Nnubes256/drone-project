pub mod quartenion;
pub mod vector;

// https://mastodon.technology/@bugaevc/102226891784062955
pub fn is_all_same<T: PartialEq>(arr: &[T]) -> bool {
    arr.windows(2).all(|w| w[0] == w[1])
}
