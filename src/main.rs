use slog::{slog_o, Drain};

mod network_initializer;

fn main() {
    /*
    let decorator = slog_term::TermDecorator::new().build();
    let drain = slog_term::FullFormat::new(decorator).build().fuse();
    let drain = slog_async::Async::new(drain).build().fuse();
    let logger = slog::Logger::root(drain, slog_o!("version" => env!("CARGO_PKG_VERSION")));

    let _scope_guard = slog_scope::set_global_logger(logger);
    slog_stdlog::init_with_level(log::Level::Info).unwrap();*/

    println!("Start!");

    network_initializer::run();

    println!("Finish!");
}
