import _ from "lodash";
import {html} from "htl";

import * as THREE from "three";
import { ThreeMFLoader } from 'three/addons/loaders/3MFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';
import { ShaderPass } from 'three/addons/postprocessing/ShaderPass.js';
import { SMAAPass } from 'three/addons/postprocessing/SMAAPass.js';
import { ClearPass } from 'three/addons/postprocessing/ClearPass.js';
import { OutlinePass } from 'three/addons/postprocessing/OutlinePass.js';
import { Sky } from 'three/addons/objects/Sky.js';


import {TimingStats} from "./utils.js";
import {FileAttachment} from "observablehq:stdlib";

const π = Math.PI;

const [model_file_url, wood_texture_url, wood_displacement_url] = await Promise.all([
  FileAttachment("../data/motor_model.3mf").url(),
  FileAttachment("../data/textures/laminate_floor_diff_1k.jpg").url(),
  FileAttachment("../data/textures/laminate_floor_disp_1k.png").url(),
]);

function create_mixing_pass(added_texture) {
  const vertexShader = `
    varying vec2 vUv;
    void main() {
      vUv = uv;
      gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
    }
  `;
  const fragmentShader = `
    uniform sampler2D base_texture;
    uniform sampler2D added_texture;
    varying vec2 vUv;
    void main() {
      gl_FragColor = ( texture2D( base_texture, vUv ) + vec4( 1.0 ) * texture2D( added_texture, vUv ) );
    }
  `;
  const mix_pass = new ShaderPass(
    new THREE.ShaderMaterial( {
      uniforms: {
        base_texture: { value: null },
        added_texture: { value: added_texture },
      },
      vertexShader, 
      fragmentShader,
      defines: {},
    } ), 'base_texture'
  );
  mix_pass.needsSwap = true;

  return mix_pass;
}

export function create_sound_wave({sample_size, max_sound_distance, start_radius, color}) {
  const max_radius = start_radius + max_sound_distance;

  const vertexShader = `
    varying vec3 vPosition;
    void main() {
      vPosition = position;
      gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
    }
  `;

  // 16384 samples is about 350us of audio at 48kHz, and sound travels a distance of 12cm during this time (at 343 m/s).
  // uniform float sound_samples[SAMPLE_SIZE];

  const fragmentShader = `
    varying vec3 vPosition;

    uniform vec3 color;
    uniform float sound_distance;
    uniform float start_radius;

    uniform sampler2D sound_samples;
    const float angular_spread = PI / 3.0;
    const float diff_scale = PI / angular_spread;

    float directional_cos(float angle_a, float angle_b) {
      float diff = angle_a - angle_b;
      float norm_diff = mod(diff + 3.0 * PI, 2.0 * PI) - PI;
      float scaled_diff = min(max(norm_diff * diff_scale, -PI*0.5), PI*0.5);
      return cos(scaled_diff);
    }

    void main() {
      float dist = distance(vPosition.xy, vec2(0.0)) - start_radius;
      
      if (dist < 0.0 || dist > sound_distance) {
        gl_FragColor = vec4(0.0);
      } else {
        float angle = atan(vPosition.x, vPosition.y);
        vec2 dist_uv = vec2(dist / sound_distance, 0.0);
        vec4 sound_sample = texture2D(sound_samples, dist_uv).rgba;
        float sound_angle = (sound_sample.g - 0.5) * 2.0 * PI;

        gl_FragColor = vec4(color, sound_sample.r * directional_cos(angle, sound_angle));
      }
    }
  `;

  const sound_data = new Uint8Array(sample_size * 4);
  const sound_samples_texture = new THREE.DataTexture(sound_data, sample_size, 1, THREE.RGBAFormat, THREE.UnsignedByteType);
  sound_samples_texture.needsUpdate = true;
  
  const sound_wave_material = new THREE.ShaderMaterial( {
    uniforms: {
      color: { value: color },
      sound_distance: { value: max_sound_distance },
      start_radius: { value: start_radius },
      sound_samples: { value: sound_samples_texture },
    },
    vertexShader,
    fragmentShader,
    defines: {PI: Math.PI},
    transparent: true,
    side: THREE.DoubleSide,
    // flatShading: true,
  });

  // Hack emissive properties onto this to add it to the bloom pass.
  sound_wave_material.emissive = new THREE.Color(0xffffff);
  sound_wave_material.emissiveIntensity = 1.0;


  const sound_wave_update = (sound_samples, sound_distance) => {

  
    const uniform_angle = function (angle) {
      return (angle / (2 * π)) + 0.5;
    };

    sound_samples.forEach(({v, φ, x, y}, i) => {
      const sample_i = 4*i;

      const intensity = v;
      const angle = uniform_angle(φ);
      
      sound_data[sample_i + 0 /* r */] = Math.floor(intensity * 255);
      sound_data[sample_i + 1 /* g */] = Math.floor(angle * 255);

      sound_data[sample_i + 2 /* b */] = Math.floor(x * 255);
      sound_data[sample_i + 3 /* a */] = Math.floor(y * 255);
    });
    sound_wave_material.uniforms.sound_distance.value = sound_distance;
    sound_samples_texture.needsUpdate = true;
  };
  
  
  const sound_wave = new THREE.Mesh(new THREE.CircleGeometry( max_radius, 64 ), sound_wave_material);
  
  sound_wave.position.z = 0;
  sound_wave.position.y = 0;
  sound_wave.position.x = 0;
  sound_wave.rotation.x = Math.PI / 2;
  
  return {sound_wave, sound_wave_update};
}


const manager = new THREE.LoadingManager();

const loader = new ThreeMFLoader(manager);
const texture_loader = new THREE.TextureLoader(manager);

function get_motor_model_components(object) {
  let stator = new THREE.Group();
  let rotor = new THREE.Group();
  
  let blue_led = new THREE.Group();
  let red_led = new THREE.Group();
  let green_led = new THREE.Group();

  let coil_U = new THREE.Group();
  let coil_V = new THREE.Group();
  let coil_W = new THREE.Group();

  let hall_sensors = new THREE.Group();
  let pin1 = new THREE.Group();
  let pin2 = new THREE.Group();
  let pin3 = new THREE.Group();

  const motor_body_to_group = {
    "Blue LED": blue_led,
    "Coil U": coil_U,
    "Coil V": coil_V,
    "Coil W": coil_W,
    "Green LED": green_led,
    "Hall sensor": hall_sensors,
    "Magnetic North": rotor,
    "Magnetic South": rotor,
    "Pin 1": pin1,
    "Pin 2": pin2,
    "Pin 3": pin3,
    "Red LED": red_led,
    "Rotor axel": rotor,
    "Rotor case": rotor,
    "Rotor gear": rotor,
    "Stator": stator,
  }

  const motor_bodies = [...object.children];
  motor_bodies.forEach(child => {
    motor_body_to_group[child.name].add(child);
  });

  const get_mesh = (group, index=0) => group.children[index].children[0];
  const get_color = (mesh) => {
    const colors = _.range(0, mesh.geometry.attributes.color.count).map(i => {
      let color = new THREE.Color()
      color.fromBufferAttribute(mesh.geometry.attributes.color, i);
      return color;
    });

    return new THREE.Color(_.meanBy(colors, c => c.r), _.meanBy(colors, c => c.g), _.meanBy(colors, c => c.b));
  };

  red_led = get_mesh(red_led);
  green_led = get_mesh(green_led);
  blue_led = get_mesh(blue_led);
  coil_U = get_mesh(coil_U);
  coil_V = get_mesh(coil_V);
  coil_W = get_mesh(coil_W);

  red_led.material.emissive = get_color(red_led);
  green_led.material.emissive = get_color(green_led);
  blue_led.material.emissive = get_color(blue_led);
  
  coil_U.material.emissive = get_color(coil_U);
  coil_V.material.emissive = get_color(coil_V);
  coil_W.material.emissive = get_color(coil_W);
  
  const hall_1 = new THREE.Group();
  const hall_2 = new THREE.Group();
  const hall_3 = new THREE.Group();

  hall_3.add(get_mesh(hall_sensors, 2));
  hall_3.add(get_mesh(pin1, 2));
  hall_3.add(get_mesh(pin2, 2));
  hall_3.add(get_mesh(pin3, 2));
  hall_3.add(blue_led);
  
  hall_2.add(get_mesh(hall_sensors, 1));
  hall_2.add(get_mesh(pin1, 1));
  hall_2.add(get_mesh(pin2, 1));
  hall_2.add(get_mesh(pin3, 1));
  hall_2.add(green_led);

  hall_1.add(get_mesh(hall_sensors, 0));
  hall_1.add(get_mesh(pin1, 0));
  hall_1.add(get_mesh(pin2, 0));
  hall_1.add(get_mesh(pin3, 0));
  hall_1.add(red_led);


  stator.add(coil_U);
  stator.add(coil_V);
  stator.add(coil_W);
  stator.add(hall_1);
  stator.add(hall_2);
  stator.add(hall_3);

  const motor = new THREE.Group();
  motor.add(stator);
  motor.add(rotor);

  return {
    motor,
    stator,
    rotor,
    blue_led,
    red_led,
    green_led,
    coil_U,
    coil_V,
    coil_W,
    hall_1,
    hall_2,
    hall_3,
  };
}

export function load_motor(model_file_url){
  return new Promise((resolve, reject) => {
    function on_load(object){
      object.traverse( function (child) {
        if ( child instanceof THREE.Mesh ) {
          child.geometry.computeBoundingSphere();
          child.receiveShadow = true;
          child.castShadow = true;
        }
      });
      resolve(get_motor_model_components(object));
    }

    function on_progress(xhr) {
      console.info(`Loading 3D motor model: ${(xhr.loaded / xhr.total * 100).toFixed(0)}%`);
    }

    function on_error(error) {
      reject(error);
    }

    // Load a 3mf file
    loader.load(model_file_url, on_load, on_progress, on_error);
  });
}

export function load_texture(texture_file_url, repeat = 1.0, color_space = THREE.NoColorSpace){
  return new Promise((resolve, reject) => {
    function on_load(texture){
      texture.wrapS = THREE.RepeatWrapping;
      texture.wrapT = THREE.RepeatWrapping;
      texture.repeat.set( repeat, repeat );
      texture.colorSpace = color_space;
      resolve(texture);
    }
    function on_progress(xhr) {
      console.info(`Loading 3D textures: ${(xhr.loaded / xhr.total * 100).toFixed(0)}%`);
    }
    function on_error(error){
      reject(error);
    }
    texture_loader.load(texture_file_url, on_load, on_progress, on_error);
  });
}

function create_camera(width, height) {
  const fov = 90;
  const aspect = width / height;
  const near = 1;
  const far = 1000;
  const camera = new THREE.PerspectiveCamera(fov, aspect, near, far);
  reset_camera_position(camera);
  return camera;
}

function reset_camera_position(camera){
  camera.position.z = 0;
  camera.position.y = -100;
  camera.position.x = 0;
  camera.up.set( 0, 0, 1 );
  camera.lookAt(0, 0, 0);
  camera.updateProjectionMatrix();
}


export async function create_scene({wave_sample_size}){
  const [motor, wood_texture, wood_displacement] = await Promise.all([
    load_motor(model_file_url),
    load_texture(wood_texture_url, 8, THREE.SRGBColorSpace),
    load_texture(wood_displacement_url, 8),
  ]);

  const scene = new THREE.Scene();

  scene.background = new THREE.Color(0xf6f6f6);

  const sky = new Sky();
  sky.scale.setScalar( 450000 );
  sky.material.uniforms.up.value.set(0, 0, 1);
  sky.material.uniforms.sunPosition.value.set(0, -200, 120)
  sky.material.uniforms.rayleigh.value = 1.0;
  sky.material.uniforms.turbidity.value = 0.5;
  sky.material.uniforms.mieCoefficient.value = 0.005;
  sky.material.uniforms.mieDirectionalG.value = 0.99;

  scene.add( sky );

  const hemi_light = new THREE.HemisphereLight( 0xffffff, 0xa0a0a0, 3);
  hemi_light.position.set( 0, -100, 100 );
  scene.add( hemi_light );

  const light = new THREE.PointLight( 0xffffff, 2, 0, 0.1);
  light.position.set( 0, -200, 120 );
  light.castShadow = true;
  //Set up shadow properties for the light
  light.shadow.mapSize.width = 512; // default
  light.shadow.mapSize.height = 512; // default
  light.shadow.camera.near = 0.5; // default
  light.shadow.camera.far = 1000; // default

  scene.add( light );

  // scene.add( new THREE.CameraHelper( dirLight.shadow.camera ) );

  const ground = new THREE.Mesh( 
    new THREE.PlaneGeometry( 4000, 4000 ), 
    new THREE.MeshPhongMaterial( { map: wood_texture, bumpMap: wood_displacement } ) 
    );
  ground.rotation.x = 0.0;
  ground.rotation.z = Math.PI / 2;
  ground.position.z = -150;
  ground.receiveShadow = true;
  scene.add( ground );

  scene.add(motor.motor);

  const {sound_wave, sound_wave_update} = create_sound_wave({
    start_radius: 55.0,
    sample_size: wave_sample_size,
    max_sound_distance: 500.0,
    color: new THREE.Color("orange"),
  });
  
  scene.add(sound_wave);

  return {motor, scene, sound_wave, sound_wave_update};
};

export function top_selection(intersected_objects){
  return intersected_objects.length > 0 ? [intersected_objects[0]] : [];
}


export const default_rendering_options = {
  width: 640, height: 640,
  highlight_filter: top_selection,
};

export function setup_rendering(scene, invalidation, options={}){
  const {width, height} = {...default_rendering_options, ...options};

  const camera = create_camera(width, height);

  const renderer = new THREE.WebGLRenderer();
  
  const stats = new TimingStats();

  renderer.domElement.addEventListener( 'pointermove', on_move );

  renderer.setSize(width, height);
  renderer.setPixelRatio(devicePixelRatio);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  // renderer.toneMapping = THREE.ReinhardToneMapping;
  // renderer.toneMappingExposure = Math.pow( 1.0, 4.0 );
  renderer.localClippingEnabled = true;

  const render_pass = new RenderPass(scene, camera);
  const clear_pass = new ClearPass();
  const output_pass = new OutputPass();
  const anti_aliasing_pass = new SMAAPass(width, height);

  const outline_pass = new OutlinePass( new THREE.Vector2( window.innerWidth, window.innerHeight ), scene, camera );
  outline_pass.edgeStrength = 4;
  outline_pass.edgeGlow = 1.0;
  outline_pass.edgeThickness = 2;
  outline_pass.pulsePeriod = 0;
  outline_pass.visibleEdgeColor.set(0xffffff);
  outline_pass.hiddenEdgeColor.set(0xffffff);

  const mouse = new THREE.Vector2();
  const raycaster = new THREE.Raycaster();

  function on_move( event ) {

    if ( event.isPrimary === false ) return;

    const rect = renderer.domElement.getBoundingClientRect();
    mouse.set(
      ( (event.clientX - rect.left) / (rect.right - rect.left) ) * 2 - 1,
      - ( (event.clientY - rect.top) / (rect.bottom - rect.top) ) * 2 + 1,
    );
  }

  function compute_highlight(highlight_filter){
    raycaster.setFromCamera( mouse, camera );

    const intersects = raycaster.intersectObject( scene, true );

    const selected = highlight_filter(intersects.map(i => i.object));
    
    outline_pass.selectedObjects = selected;
  }

  const bloom_params = {
    threshold: 0.0,
    strength: 1.0,
    radius: 1,
  };

  const dark_material = new THREE.MeshBasicMaterial( { color: 'black' } );

  const bloom_pass = new UnrealBloomPass(
    new THREE.Vector2(width, height), 
    bloom_params.strength, 
    bloom_params.radius, 
    bloom_params.threshold,
  );

  const bloom_composer = new EffectComposer(renderer);
  bloom_composer.renderToScreen = false;
  bloom_composer.addPass(render_pass);
  bloom_composer.addPass(bloom_pass);

  const mix_pass = create_mixing_pass(bloom_composer.renderTarget2.texture);

  const composer = new EffectComposer(renderer);
  composer.addPass(render_pass);
  // composer.addPass(clear_pass);
  composer.addPass(mix_pass);
  composer.addPass(outline_pass);
  composer.addPass(output_pass);
  composer.addPass(anti_aliasing_pass);


  const controls = new OrbitControls( camera, renderer.domElement );
  controls.minDistance = 50;
  controls.maxDistance = 500;
  controls.enablePan = true;
  controls.enableZoom = true;
  controls.maxPolarAngle = Math.PI * (3 / 4);
  controls.minPolarAngle = Math.PI * (1 / 4);
  controls.target.set( 0, 0, 0 );


  function reset_camera(){
    reset_camera_position(camera);
    controls.target.set( 0, 0, 0 );
    controls.update();
  }

  invalidation.then(() => {
    renderer.domElement.removeEventListener( 'pointermove', on_move );
    composer.dispose();
    bloom_composer.dispose();
    controls.dispose();
    renderer.dispose();
  });
  
  function is_emissive(material){
    if (material.emissive === undefined) return false;
    return (material.emissive.r > 0 || material.emissive.g > 0 || material.emissive.b > 0) && material.emissiveIntensity > 0;
  }

  function render(options={}){
    const {highlight_filter} = {...default_rendering_options, ...options};

    controls.update();

    // Get the mouseover item to highlight if any.
    compute_highlight(highlight_filter);


    // Darken the scene and render the bloom pass for emissive materials.
    const materials = {};
    const lights = {};
    const saved_background = scene.background;
    
    scene.traverse( (obj) => {
      // Darken every non-emissive material.
      if ((obj instanceof THREE.Mesh) && !is_emissive(obj.material)){
        materials[obj.uuid] = obj.material;
        obj.material = dark_material;
      }
      // Turn off all lights.
      if ((obj instanceof THREE.Light)){
        lights[obj.uuid] = {intensity: obj.intensity};
        obj.intensity = 0;
      }
    } );
    // Darken the background.
    scene.background = new THREE.Color(0x000000);
    // Render the glowing emissive objects.
    bloom_composer.render();

    // Restore the scene.
    scene.traverse( (obj) =>{
      // Recover materials to their original state.
      if (materials[obj.uuid]){
        obj.material = materials[obj.uuid];
        delete materials[obj.uuid];
      }
      // Recover lights to their original state.
      if (lights[obj.uuid]){
        obj.intensity = lights[obj.uuid].intensity;
        delete lights[obj.uuid];
      }
    });
    // Recover the background.
    scene.background = saved_background;
    
    // Render the whole scene using the composer to mix the normal scene and the bloom pass.
    composer.render();

    stats.update();
  }


  const div = html`<div class="layered">${renderer.domElement}</div>`;

  return {
    canvas: renderer.domElement,
    stats,
    div,
    render,
    camera,
    reset_camera,
  };
}
