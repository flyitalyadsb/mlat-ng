# Build stage pinned to the same toolchain as rust-toolchain.toml.
FROM docker.io/library/rust:1.92-bookworm AS build
WORKDIR /src
COPY Cargo.toml Cargo.lock rust-toolchain.toml ./
# Dependency layer: build once with a dummy main so source edits don't
# recompile tokio/nalgebra every time.
RUN mkdir src && echo 'fn main() {}' > src/main.rs && cargo build --release --locked && rm -rf src
COPY src ./src
RUN touch src/main.rs && cargo build --release --locked && cp target/release/mlat-ng /mlat-ng

FROM gcr.io/distroless/cc-debian12:nonroot
COPY --from=build /mlat-ng /usr/local/bin/mlat-ng
USER nonroot
EXPOSE 30100 31003 9100
ENTRYPOINT ["/usr/local/bin/mlat-ng"]
