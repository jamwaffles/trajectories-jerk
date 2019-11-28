# Jerk Trajectories Example 1: one-dimensional trapezoidal trajectory.

## Setup

- Follow [the setup guide](https://rustwasm.github.io/docs/book/game-of-life/setup.html)

- Install [http-server](https://www.npmjs.com/package/http-server) to load demos in your web browser (required due to CORS issues).

  ```bash
  npm i -g http-server
  ```

## Build it

```bash
wasm-pack build --target web
```

## Run it

```bash
http-server --cors
```

Open <http://localhost:8081> in your browser.
