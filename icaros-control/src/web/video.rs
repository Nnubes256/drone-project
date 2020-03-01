use tokio::prelude::*;
use tokio_util::codec::Decoder;
use bytes::Buf;
use std::iter::FromIterator;

const NAL_SEPARATOR: &[u8] = &[0,0,0,1];

lazy_static! {
    static ref NAL_SEPARATOR_REGEX: regex::bytes::Regex = {
        regex::bytes::Regex::new(r"(?-u)\x00\x00\x00\x01").unwrap()
    };
}

pub struct NALSeparator {
    offset: usize,
    _read: usize,
}

pub fn separate_nal() -> NALSeparator
{
    NALSeparator {
        offset: 0,
        _read: 0,
    }
}

/*macro_rules! cur_buff {
    ($me: ident) => { if *$me.using_second_buffer { &mut $me.buffer2 } else { &mut $me.buffer1 } }
}

impl<T: AsyncBufRead> Stream for NALSeparator<T> {
    type Item = Result<Vec<u8>, io::Error>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        let mut me = self.project();
        me.cache.clear();

        *me._read = match read_until_internal(me.reader, cx, 0x01, &mut me.cache, &mut me._read) {
            Poll::Ready(_) if *me._read == 0 && me.cache.len() == 0 => return Poll::Ready(None),
            Poll::Ready(n) => n,
            Poll::Pending => return Poll::Pending
        }?;
        let cache_len = *me._read;

        let mut current_buffer = cur_buff!(me);

        if *me.offset + cache_len > *me.capacity + *me.buffer_flush {
            let minimal_length = *me.capacity - *me.body_offset + cache_len;

            if *me.capacity < minimal_length {
                *me.capacity = minimal_length;
                me.buffer1.resize(*me.capacity, 0);
                me.buffer2.resize(*me.capacity, 0);
            }

            *me.using_second_buffer = !*me.using_second_buffer;
            let previous_buffer;
            if *me.using_second_buffer {
                current_buffer = &mut me.buffer1;
                previous_buffer = &mut me.buffer2;
            } else {
                current_buffer = &mut me.buffer2;
                previous_buffer = &mut me.buffer1;
            }
            current_buffer[..*me.body_offset].copy_from_slice(&previous_buffer[..*me.body_offset]);
            *me.offset -= *me.body_offset;
            *me.body_offset = 0;
        }

        current_buffer[*me.offset..(*me.offset + &me.cache.len())].copy_from_slice(&me.cache);

        let ret: Poll<Option<Self::Item>>;
        let start = cmp::max(*me.body_offset, me.offset.saturating_sub(4));
        let stop = *me.offset + cache_len;

        if let Some(idx) = NAL_SEPARATOR_REGEX.find(&current_buffer[start..stop]) {
            let idx2 = idx.start() + start;
            let frame: Vec<u8> = Vec::from_iter(
                NAL_SEPARATOR.iter().chain(&mut current_buffer[*me.body_offset..idx2].iter()).copied());
            ret = Poll::Ready(Some(Ok(frame)));
            *me.body_offset = idx2 + 4;
        } else {
            ret = Poll::Pending;
        }

        *me.offset += cache_len;

        return ret;
    }
}*/

impl Decoder for NALSeparator {
    type Item = Vec<u8>;
    type Error = io::Error;

    fn decode(&mut self, data: &mut bytes::BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        let index;
        if let Some(idx) = NAL_SEPARATOR_REGEX.find(&data[self.offset..]) {
            index = idx.end();
        } else {
            self.offset = data.len().saturating_sub(4);
            return Ok(None);
        }

        let final_data = Vec::from_iter(NAL_SEPARATOR.iter().copied().chain(data.split_to(index - 4)));
        data.advance(4);
        self.offset = 0;
        Ok(Some(final_data))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;
    use tokio_util::codec::FramedRead;
    use futures::StreamExt;

    #[tokio::test]
    async fn nal_separator() {
        let some_stuff = Cursor::new(vec![3, 4, 5, 6, 7, 8, 9, 10, 0, 0, 0, 1, 7, 8, 9, 0, 0, 0, 1]);
        let expected = vec![
            vec![0, 0, 0, 1, 3, 4, 5, 6, 7, 8, 9, 10],
            vec![0, 0, 0, 1, 7, 8, 9]
        ];

        let nal_separator = FramedRead::new(some_stuff, separate_nal());

        let mut i = 0;

        let mut stream = nal_separator
        .map(|x| {
            assert_ne!(i, 2);
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
