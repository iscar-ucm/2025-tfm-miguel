use core::f64;

use xdevs::modeling::*;
use xdevs::simulation::*;

#[derive(Clone, Debug)]
struct Job {
    time: f64,
    name: String,
}

impl Job {
    fn new(name: &str, time: f64) -> Self {
        Job {
            time,
            name: name.to_string(),
        }
    }
}

struct Generator {
    component: Component,
    i_stop: InPort<bool>,
    o_job: OutPort<Job>,
    sigma: f64,
    time: f64,
}

impl Generator {
    fn new(name: &str, time: f64) -> Self {
        let mut component = Component::new(name);
        let i = component.add_in_port::<bool>("i_stop");
        let o = component.add_out_port::<Job>("o_job");
        Generator {
            component,
            i_stop: i,
            o_job: o,
            sigma: 1.0,
            time: time,
        }
    }
}

impl Atomic for Generator {
    fn get_component(&self) -> &Component {
        &self.component
    }

    fn get_component_mut(&mut self) -> &mut Component {
        &mut self.component
    }

    fn lambda(&self) {
        let job = Job::new("Job Generated", self.get_t_last());
        unsafe { self.o_job.add_value(job.clone()) };
        println!("{}: {} - {}", self.get_name(), job.name, self.get_t_last());
    }

    fn delta_int(&mut self) {
        self.sigma = self.time;
    }

    fn delta_ext(&mut self, e: f64) {
        self.sigma -= e;
        if !unsafe { self.i_stop.is_empty() } {
            self.sigma = f64::INFINITY;
        }
    }

    fn ta(&self) -> f64 {
        self.sigma
    }
}

struct Processor {
    component: Component,
    i_job: InPort<Job>,
    o_job: OutPort<Job>,
    sigma: f64,
    time: f64,
    job: Option<Job>,
}

impl Processor {
    fn new(name: &str, time: f64) -> Self {
        let mut component = Component::new(name);
        let i = component.add_in_port::<Job>("i_job");
        let o = component.add_out_port::<Job>("o_job");
        Processor {
            component,
            i_job: i,
            o_job: o,
            sigma: f64::INFINITY,
            time: time,
            job: None,
        }
    }
}

impl Atomic for Processor {
    fn get_component(&self) -> &Component {
        &self.component
    }

    fn get_component_mut(&mut self) -> &mut Component {
        &mut self.component
    }

    fn lambda(&self) {
        let job = Job::new("Job Processed", self.job.as_ref().unwrap().time);
        unsafe { self.o_job.add_value(job.clone()) };
        println!(
            "{}: {} - {}",
            self.get_name(),
            job.name,
            self.job.as_ref().unwrap().time
        );
    }

    fn delta_int(&mut self) {
        self.sigma = f64::INFINITY;
        self.job = None;
    }

    fn delta_ext(&mut self, e: f64) {
        self.sigma -= e;

        if self.job.is_none() {
            self.job = unsafe { self.i_job.get_values().first().cloned() };
            self.sigma = self.time
        }
    }

    fn ta(&self) -> f64 {
        self.sigma
    }
}

struct Transducer {
    component: Component,
    i_arrived_job: InPort<Job>,
    i_solved_job: InPort<Job>,
    o_stop: OutPort<bool>,
    arrived_job: Vec<Job>,
    solved_job: Vec<Job>,
    sigma: f64,
}

impl Transducer {
    fn new(name: &str, time: f64) -> Self {
        let mut component = Component::new(name);
        let a = component.add_in_port::<Job>("i_arrived_Job");
        let s = component.add_in_port::<Job>("i_solved_Job");
        let o = component.add_out_port::<bool>("o_stop");
        Transducer {
            component,
            i_arrived_job: a,
            i_solved_job: s,
            arrived_job: Vec::new(),
            solved_job: Vec::new(),
            o_stop: o,
            sigma: time,
        }
    }
}

impl Atomic for Transducer {
    fn get_component(&self) -> &Component {
        &self.component
    }

    fn get_component_mut(&mut self) -> &mut Component {
        &mut self.component
    }

    fn lambda(&self) {
        unsafe { self.o_stop.add_value(true) };
    }

    fn delta_int(&mut self) {
        self.sigma = f64::INFINITY;
        println!("Stopped!");
    }

    fn delta_ext(&mut self, e: f64) {
        self.sigma -= e;
        if !unsafe { self.i_arrived_job.is_empty() } {
            self.arrived_job
                .extend_from_slice(unsafe { self.i_arrived_job.get_values() });
        }
        if !unsafe { self.i_solved_job.is_empty() } {
            self.solved_job
                .extend_from_slice(unsafe { self.i_solved_job.get_values() });
        }
    }

    fn ta(&self) -> f64 {
        self.sigma
    }
}

struct EF {
    coupled: Coupled,
}

impl EF {
    fn new(name: &str, generate: f64, analyze: f64) -> Self {
        let mut coupled = Coupled::new(name);
        coupled.add_in_port::<Job>("i_job");
        coupled.add_out_port::<Job>("o_job");

        let g = Generator::new("Generator", generate);
        let t = Transducer::new("Transducer", analyze);

        coupled.add_component(Box::new(g));
        coupled.add_component(Box::new(t));

        coupled.add_ic("Generator", "o_job", "Transducer", "i_arrived_Job");
        coupled.add_ic("Transducer", "o_stop", "Generator", "i_stop");
        coupled.add_eic("i_job", "Transducer", "i_solved_Job");
        coupled.add_eoc("Generator", "o_job", "o_job");

        Self { coupled }
    }
}

fn efp(g_time: f64, p_time: f64, t_time: f64) -> Coupled {
    let mut efp = Coupled::new("EFP");
    let ef = EF::new("EF", g_time, t_time);
    let p = Processor::new("Processor", p_time);

    efp.add_component(Box::new(ef.coupled));
    efp.add_component(Box::new(p));

    efp.add_ic("EF", "o_job", "Processor", "i_job");
    efp.add_ic("Processor", "o_job", "EF", "i_job");

    efp
}

fn main() {
    let efp = efp(1.0, 3.0, 100.0);
    let mut simulator = RootCoordinator::new(efp);
    simulator.simulate(f64::INFINITY);
}
