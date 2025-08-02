Motor Simulator & Control
-------------------------

The motor control page and an explanatory simulator are built using the [Observable Framework](https://observablehq.com/framework). 
To start the local preview server, run:

```
npm run dev
```

Then visit <http://localhost:3000> to preview your app.


Webapp structure
----------------

The main entry points for the motor simulator and driver controller are listed below:

```ini
.
├─ src
│  ├─ components
│  │  ├─ motor_controller.js   # Code to connect to the motor (will be extracted into a separate lib eventually).
│  │  ├─ motor_interface       # Motor driver API. This is the USB interface, which will be kept consistent with I2C, SPI and CAN interfaces.
│  │  └─ simulations.js        # Code driving the motor simulator.
│  ├─ data
│  │  └─ motor_model.3mf       # An example motor 3D model.
│  │  
│  ├─ index.md                 # Homepage!
│  ├─ motor_monitor.md         # Motor control page! (Use this page as reference for connecting to the motor programmatically.)
│  ├─ motor_model.md           # Physics equation describing the 3 phase motor electrical relations.
│  ├─ normal_dist_tricks.md    # Illustration for combining Gaussian distributions and for the Kalman filter position updates.
│  └─ three_phase_tricks.md    # Tricks and trigonometric tables to use for driving the motor with the maximum voltage range.
│
├─ .gitignore
├─ observablehq.config.js      # The app config file.
├─ package.json
└─ README.md
```



For reference, the typical Framework project looks like this:

- `src`: This is the “source root” — where your source files live. Pages go here. Each page is a Markdown file. Observable Framework uses [file-based routing](https://observablehq.com/framework/routing), which means that the name of the file controls where the page is served. You can create as many pages as you like. Use folders to organize your pages.
- `src/index.md`: This is the home page for your app. You can have as many additional pages as you’d like, but you should always have a home page, too.
- `src/data`: You can put [data loaders](https://observablehq.com/framework/loaders) or static data files anywhere in your source root, but we recommend putting them here.
- `src/components`: You can put shared [JavaScript modules](https://observablehq.com/framework/javascript/imports) anywhere in your source root, but we recommend putting them here. This helps you pull code out of Markdown files and into JavaScript modules, making it easier to reuse code across pages, write tests and run linters, and even share code with vanilla web applications.
- `observablehq.config.js`: This is the [app configuration](https://observablehq.com/framework/config) file, such as the pages and sections in the sidebar navigation, and the app’s title.


Command reference
-----------------

| Command           | Description                                              |
| ----------------- | -------------------------------------------------------- |
| `npm install`            | Install or reinstall dependencies                        |
| `npm run dev`        | Start local preview server                               |
| `npm run build`      | Build your static site, generating `./dist`              |
| `npm run deploy`     | Deploy your app to Observable                            |
| `npm run clean`      | Clear the local data loader cache                        |
| `npm run observable` | Run commands like `observable help`                      |
