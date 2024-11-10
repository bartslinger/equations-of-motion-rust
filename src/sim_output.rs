use crate::rigid_body::State;
use wasm_bindgen::prelude::wasm_bindgen;

#[derive(Clone)]
#[wasm_bindgen]
pub struct SimOutput {
    pub(crate) time: Vec<f64>,
    pub(crate) states: Vec<State>,
}

#[wasm_bindgen]
impl SimOutput {
    pub fn new() -> Self {
        Self {
            time: Vec::new(),
            states: Vec::new(),
        }
    }

    #[wasm_bindgen]
    pub fn time(&self) -> Vec<f64> {
        self.time.clone()
    }

    #[wasm_bindgen]
    pub fn u(&self) -> Vec<f64> {
        self.states.iter().map(|state| state[0]).collect()
    }

    #[wasm_bindgen]
    pub fn v(&self) -> Vec<f64> {
        self.states.iter().map(|state| state[1]).collect()
    }

    #[wasm_bindgen]
    pub fn w(&self) -> Vec<f64> {
        self.states.iter().map(|state| state[2]).collect()
    }

    #[wasm_bindgen]
    pub fn p(&self) -> Vec<f64> {
        self.states.iter().map(|state| state[3]).collect()
    }

    #[wasm_bindgen]
    pub fn q(&self) -> Vec<f64> {
        self.states.iter().map(|state| state[4]).collect()
    }

    #[wasm_bindgen]
    pub fn r(&self) -> Vec<f64> {
        self.states.iter().map(|state| state[5]).collect()
    }

    #[wasm_bindgen]
    pub fn q0(&self) -> Vec<f64> {
        self.states.iter().map(|state| state[6]).collect()
    }

    #[wasm_bindgen]
    pub fn q1(&self) -> Vec<f64> {
        self.states.iter().map(|state| state[7]).collect()
    }

    #[wasm_bindgen]
    pub fn q2(&self) -> Vec<f64> {
        self.states.iter().map(|state| state[8]).collect()
    }

    #[wasm_bindgen]
    pub fn q3(&self) -> Vec<f64> {
        self.states.iter().map(|state| state[9]).collect()
    }

    #[wasm_bindgen]
    pub fn pn(&self) -> Vec<f64> {
        self.states.iter().map(|state| state[10]).collect()
    }

    #[wasm_bindgen]
    pub fn pe(&self) -> Vec<f64> {
        self.states.iter().map(|state| state[11]).collect()
    }

    #[wasm_bindgen]
    pub fn pd(&self) -> Vec<f64> {
        self.states.iter().map(|state| state[12]).collect()
    }
}
