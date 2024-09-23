import {FileAttachment} from "npm:@observablehq/stdlib";

import * as THREE from "three";
import { ThreeMFLoader } from 'three/addons/loaders/3MFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { ArcballControls } from 'three/addons/controls/ArcballControls.js';

const manager = new THREE.LoadingManager();

const loader = new ThreeMFLoader(manager);

function group_motor_model(object) {
  let stator = new THREE.Group();
  let rotor = new THREE.Group();

  const motor_body_to_group = {
    "Blue LED": stator,
    "Coil U": stator,
    "Coil V": stator,
    "Coil W": stator,
    "Green LED": stator,
    "Hall sensor": stator,
    "Magnetic North": rotor,
    "Magnetic South": rotor,
    "Pin 1": stator,
    "Pin 2": stator,
    "Pin 3": stator,
    "Red LED": stator,
    "Rotor axel": rotor,
    "Rotor case": rotor,
    "Rotor gear": rotor,
    "Stator": stator,
  }

  const motor_bodies = [...object.children];
  motor_bodies.forEach(child => {
    motor_body_to_group[child.name].add(child);
  });

  return {stator, rotor};
}

export function load_motor(model_file_url){
  return new Promise((resolve, reject) => {
    function on_load(object){
      object.traverse( function (child) {
        child.castShadow = true;
      });
      resolve(group_motor_model(object));
    }

    function on_progress(xhr) {
      console.log((xhr.loaded / xhr.total * 100) + '% loaded');
    }

    function on_error(error) {
      reject(error);
    }

    // Load a 3mf file
    loader.load(model_file_url, on_load, on_progress, on_error);
  });
}

function create_camera(width, height) {
  const fov = 90;
  const aspect = width / height;
  const near = 1;
  const far = 5000;
  const camera = new THREE.PerspectiveCamera(fov, aspect, near, far);
  camera.position.z = 0;
  camera.position.y = -100;
  camera.position.x = 0;
  camera.lookAt(0, 0, 0);
  camera.updateProjectionMatrix();
  return camera;
}


export function create_scene(motor){
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0xf0f0f0);

  const hemiLight = new THREE.HemisphereLight( 0xffffff, 0x8d8d8d, 3 );
  hemiLight.position.set( 0, 100, 0 );
  scene.add( hemiLight );

  const dirLight = new THREE.DirectionalLight( 0xffffff, 3 );
  dirLight.position.set( - 0, 40, 50 );
  dirLight.castShadow = true;
  dirLight.shadow.camera.top = 50;
  dirLight.shadow.camera.bottom = - 25;
  dirLight.shadow.camera.left = - 25;
  dirLight.shadow.camera.right = 25;
  dirLight.shadow.camera.near = 0.1;
  dirLight.shadow.camera.far = 200;
  dirLight.shadow.mapSize.set( 1024, 1024 );
  scene.add( dirLight );

  // scene.add( new THREE.CameraHelper( dirLight.shadow.camera ) );

  const ground = new THREE.Mesh( 
    new THREE.PlaneGeometry( 1000, 1000 ), 
    new THREE.MeshPhongMaterial( { color: 0xcbcbcb, depthWrite: false } ) 
    );
  ground.rotation.x = 0.0;//- Math.PI / 2;
  ground.position.z = -80;
  ground.receiveShadow = true;
  scene.add( ground );

  
  scene.add(motor.stator);
  scene.add(motor.rotor);
  return scene;
};



export function* render_scene(width, height, invalidation, scene, update) {
  const camera = create_camera(width, height);

  const renderer = new THREE.WebGLRenderer({antialias: true});
  
  renderer.setSize(width, height);
  renderer.setPixelRatio(devicePixelRatio);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  const controls = {
    "arcball": () => {
        let controls = new ArcballControls( camera, renderer.domElement, scene );
        controls.setGizmosVisible(true);
        controls.enablePan = false;
        return controls;
    },
    "orbit": () => {
        let controls = new OrbitControls( camera, renderer.domElement );
        controls.minDistance = 50;
        controls.maxDistance = 200;
        controls.enablePan = true;
        controls.target.set( 0, 0, 0 );
        // controls.maxPolarAngle = 0; // Math.PI / 2;
        // controls.minPolarAngle = 0; // -Math.PI / 2;
        controls.maxAzimuthAngle = 0; // Math.PI / 2;
        controls.minAzimuthAngle = 0; // -Math.PI / 2;
        return controls;
    }
  }["arcball"]();
  


  invalidation.then(() => {
    controls.dispose();
    renderer.dispose();
  });

  while (true) {
    update();
    controls.update();
    renderer.render(scene, camera);
    yield renderer.domElement;
  }
}
