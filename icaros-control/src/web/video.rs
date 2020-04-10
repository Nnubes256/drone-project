//! Video processing for the web side
//!
//! # Background
//!
//! A H.264 video stream is composed of NAL units (NALU). Each NAL unit starts with
//! `[0x00, 0x00, 0x00, 0x01]`[^1]. Our only objective is to split these unit using
//! this delimiter to get the individual NAL units, which the web server can then
//! send to its clients.
//!
//! [^1]: [https://yumichan.net/video-processing/video-compression/introduction-to-h264-nal-unit/](https://yumichan.net/video-processing/video-compression/introduction-to-h264-nal-unit/)

use tokio::prelude::*;
use tokio_util::codec::Decoder;
use bytes::Buf;
use std::iter::FromIterator;
use serde::Serialize;

/// The byte combination that delimits H.264 NAL units.
const NAL_SEPARATOR: &[u8] = &[0,0,0,1];

lazy_static! {
    /// A regular expression which can find the byte combination that
    /// delimits H.264 NAL units on a byte slice.
    static ref NAL_SEPARATOR_REGEX: regex::bytes::Regex = {
        regex::bytes::Regex::new(r"(?-u)\x00\x00\x00\x01").unwrap()
    };
}

///
/// The different payloads that comphrend the basic protocol used by the front-end
/// to configure the stream.
#[derive(Serialize)]
#[serde(tag = "action", content = "payload")]
pub enum VideoControlPayload {
    #[serde(rename = "initialize")]
    Initialize { width: u32, height: u32, stream_active: bool },
    #[serde(rename = "stream_active")]
    StreamActive(bool)
}

///
/// A decoder which takes a H.264 bitstream and outputs a series of framed
/// bitstreams corresponding to the individual NAL units extracted from it.
pub struct NALSeparator {
    offset: usize,
    _read: usize,
}

///
/// Creates a decoder which which takes a H.264 bitstream and outputs a series of
/// framed bitstreams corresponding to the individual NAL units extracted from it.
pub fn separate_nal() -> NALSeparator
{
    NALSeparator {
        offset: 0,
        _read: 0,
    }
}

impl Decoder for NALSeparator {
    type Item = Vec<u8>;
    type Error = io::Error;

    fn decode(&mut self, data: &mut bytes::BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        // We are given `data` as the data we have to chunk into NAL units
        // If we return `Ok(None)`, we don't have any complete unit yet, so on the next
        // call we will get the same data on top of any additional received data.
        // If we find that we have complete data, we are responsible of removing the read
        // bytes from the `data` buffer, and return the final chunked NAL frame.

        let index;

        // Attempt to find the NAL unit separator in the data
        if let Some(idx) = NAL_SEPARATOR_REGEX.find_at(&data, self.offset) {
            // We have it! Store its start index.
            index = idx.start();
        } else {
            // We don't have it, so we wait until next time, continuing the search
            // at where we left off (minus some margin).
            self.offset = data.len().saturating_sub(4);
            return Ok(None);
        }

        // We split the data buffer by the start index of the found separator, removing
        // the data we will output and leaving the rest of the data (which will be part
        // of the next NAL unit)
        let data_split = data.split_to(index);
        data.advance(4); // Then we discard the separator itself

        // It makes no sense to return an empty NAL unit, so if we find that our output
        // will be empty, we simply discard it and act as if the unit was not there.
        if data_split.is_empty() {
            return Ok(None);
        }

        // For compatibility purposes, we append the NAL separator itself to the beginning
        // of the outputted NAL unit.
        let final_data = Vec::from_iter(NAL_SEPARATOR.iter().copied().chain(data_split));

        // We reset our starting location for searches
        self.offset = 0;

        // And finally return the extracted NAL unit
        Ok(Some(final_data))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;
    //use tokio::fs::File;
    use tokio_util::codec::FramedRead;
    use futures::StreamExt;
    //use futures::TryFutureExt;

    #[tokio::test]
    async fn nal_separator() {
        //let some_stuff = File::open("test.mp4").await.unwrap();
        let some_stuff = Cursor::new(
            vec![0, 0, 0, 1, 3, 4, 5, 6, 7, 8, 9, 10,
                 0, 0, 0, 1, 7, 8, 9, 0, 0, 0, 1, 5, 6, 7, 8, 9, 10, 0, 0, 0, 1]
        );
        let expected: Vec<Vec<u8>> = vec![
            vec![0, 0, 0, 1, 3, 4, 5, 6, 7, 8, 9, 10],
            vec![0, 0, 0, 1, 7, 8, 9],
            vec![0, 0, 0, 1, 5, 6, 7, 8, 9, 10],
        ];

        let nal_separator = FramedRead::new(some_stuff, separate_nal());

        let mut i = 0;

        let mut stream = nal_separator
        .map(|x| {
            assert_ne!(i, 3);
            let data = match x {
                Ok(data) => data,
                Err(err) => return Err(err)
            };
            println!("{:?}", data);
            println!("{:?}", expected[i]);
            assert_eq!(expected[i].iter().zip(&data).all(|(a, b)| (*a == *b)), true);
            i += 1;
            Ok(data)
        });

        while let Some(_) = stream.next().await {}
    }
}
