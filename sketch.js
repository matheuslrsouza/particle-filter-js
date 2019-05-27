window['DEBUG'] = true

let ray
let walls
let robot
let particles
let nParticles = 2000

function setup() {

  createCanvas(600, 400)
  background(220)

  //robot = new Robot(createVector(random() * width, random() * height), 0)
  robot = new Robot(createVector(100, 130), 0)
  ray = new Ray(createVector(0, 0))
  walls = []

  for (mapData of mapData) {
    walls.push(new Wall(createVector(mapData.a.x, mapData.a.y), createVector(mapData.b.x, mapData.b.y)))
  }

  //outside borders
  walls.push(new Wall(createVector(0, 0), createVector(0, height)))
  walls.push(new Wall(createVector(0, 0), createVector(width, 0)))
  walls.push(new Wall(createVector(width, 0), createVector(width, height)))
  walls.push(new Wall(createVector(0, height), createVector(width, height)))

  particles = []
  for (let i = 0; i < nParticles; i++) {
    particles.push(new Particle(random() * width, random() * height, randomGaussian(robot.heading(), 0.0005)))
  }

  //particles.push(new Particle(robot.pos.x, robot.pos.y + 0, robot.heading()))
  //particles.push(new Particle(robot.pos.x, robot.pos.y + 100, robot.heading()))
  //particles.push(new Particle(robot.pos.x, robot.pos.y + 200, robot.heading()))
  

  // frameRate(10)
  // noLoop()


}

let vel = 50
let delta_t = 0.1
let yaw_rate
// x, y, theta
let std_pos = [3, 3, 0.05]
const std_landmark = [60, 60]


function draw() {

  background(0)

  yaw_rate = 0.09
  // yaw_rate = 0
  robot.show()
  
  
  //draw the walls
  for(wall of walls) {
    wall.show()
  }
  
  let measurements = robot.check(walls)

  //temp melhorar para filter
  const weights = []

  for (particle of particles) {
    particle.check(walls, measurements)
    particle.predict(delta_t, std_pos, vel, yaw_rate)
    particle.show()

    weights.push(particle.updateWeights(measurements, std_landmark))

  }

  robot.move(vel, delta_t, yaw_rate)

  //resample
  const maxW = Math.max.apply(null, weights)
  let beta = 0.0
  const N = weights.length
  let index = parseInt(random() * N)

  let selectedParticles = []

  let meanX = 0
  let meanY = 0

  let sigmaX = 0
  let sigmaY = 0
  
  for (let i = 0; i < N; i++) {
    beta += random() * 2.0 * maxW
    while (beta > weights[index]) {
      beta -= weights[index]
      index = (index + 1) % N
    }
    const p = particles[index]
    selectedParticles.push(new Particle(p.pos.x, p.pos.y, p.dir.heading()))
    
    //debug
    meanX += p.pos.x / N
    meanY += p.pos.y / N
    
    let sigmaSquareX = Math.pow(p.pos.x - meanX, 2)
    let sigmaSquareY = Math.pow(p.pos.y - meanY, 2)

    sigmaX += sigmaSquareX
    sigmaY += sigmaSquareY
  }

  particles = selectedParticles

  //debug
  let stddevX = Math.sqrt(sigmaX * (1 / N))
  let stddevY = Math.sqrt(sigmaY * (1 / N))

  push()
  fill(0, 0, 255, 20)
  ellipse(meanX, meanY, Math.log(stddevX) * 8, Math.log(stddevY) * 8)
  pop()
}

function mousePressed() {
  redraw()
}