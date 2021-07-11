use std::num::Wrapping;
use num::Integer;
use std::ops::Add;

fn hash64(input: u64) -> u64 {
    // From Murmer
    input ^= input >> 16;
    input *= 0x85EBCA6B;
    input ^= input >> 13;
    input *= 0xC2B2AE35;
    input >> 16
}

type RandomIndex<T> = Wrapping<T>;

pub trait SeededRandom<DataType, IndexType, SeedType>
where
    DataType: Default,
    IndexType: Integer + From<usize>,
    SeedType: Default,
{
    fn new() -> Self;
    fn set_seed(&mut self, seed: SeedType);
    fn get_random(&self, index: IndexType) -> DataType;
}

pub trait Random2D<DataType, IndexType, SeedType>: SeededRandom<DataType, IndexType, SeedType> {
    fn get_random_2d(&self, x: IndexType, y: IndexType) -> DataType;
}

struct HashedRandom2D16Bit {
    seed: u64,
}

impl SeededRandom<u16, u64, u64> for HashedRandom2D16Bit {
    fn new() -> Self {
        HashedRandom2D16Bit {
            seed: u64::default(),
        }
    }
    
    fn set_seed(&mut self, seed: u64) {
        self.seed = seed;
    }

    fn get_random(&self, index: u64) -> u16 {
        let remain = index & 0x3;
        ((hash64(index / 4) >> (remain * 16)) & 0xFFFF) as u16
    }
}

impl Random2D<u16, u64, u64> for HashedRandom2D16Bit {
    fn get_random_2d(&self, x: u64, y: u64) -> u16 {
        let side_length = 1 << 34;
        self.get_random((y % side_length) * side_length + (x % side_length))
    }
}

pub struct RandomSlice2D<T: Random2D> {
    width: usize,
    height: usize,
    generator: T,
    buffer: Vec<T::RandData>,
}

impl<T: Random2D> RandomSlice2D<T> {
    pub fn new(width: usize, height: usize, seed: T::SeedType) -> Self {
        let new_gen = T::new();
        new_gen.set_seed(seed);
        RandomSlice2D {
            width: width,
            height: height,
            generator: new_gen,
            buffer: vec![T::RandData::default(); width * height],
        }
    }

    pub fn get_value(&self, x: usize, y: usize) -> T::RandData {
        self.buffer[y * self.width + x]
    }

    pub fn get_slice_at(&mut self, x: T::IndexType, y: T::IndexType) {
        let x_wrapped = Wrapping(x);
        let y_wrapped = Wrapping(y);
        for dy in 0..self.height {
            for dx in 0..self.width {
                self.buffer[dy * self.width + dx] = self.generator.get_random_2d(x_wrapped + dx, y_wrapped + dy);
            }
        }
    }

    pub fn update_slice_dimentions(&mut self, new_width: usize, new_height: usize) {
        self.width = new_width;
        self.height = new_height;
        self.buffer.resize(new_width * new_height, 0);
    }

    pub fn get_critical_points(&self, desired_number: u32) -> Vec<(usize, usize)> {
        let point_buffer: Vec<(usize, usize)> = Vec::new();
        let density: f32 = desired_number as f32 / (self.width * self.height) as f32;
        let comp_val = ((1 << 16) as f32 * density) as u16;
        for y in 0..self.height {
            for x in 0..self.width {
                let rand_val = self.buffer[y * self.width + x];
                if rand_val < comp_val || rand_val > (1 << 16) - comp_val {
                    point_buffer.push((x, y));
                }
            }
        }
        point_buffer
    }
}