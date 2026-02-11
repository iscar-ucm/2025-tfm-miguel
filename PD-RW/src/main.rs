mod discrete_time_model;
mod plotters;

use discrete_time_model::DiscreteTimeModel;
use xdevs::simulation::*;

fn main() {
    let total_time = 100.0;
    let model = DiscreteTimeModel::new("DiscreteTimeModel", Some(0.01));
    let mut simulator = RootCoordinator::new(model.coupled);
    simulator.simulate(total_time);

    let transducer = unsafe { &*model.transducer_ref };
    plotters::draw(transducer, total_time);
}
