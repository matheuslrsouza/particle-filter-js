
let ray
let walls
let robot
let particles
let nParticles = 2000

let xoff = 0
let yoff = 2000

function setup() {

  createCanvas(600, 400)
  background(220)

  robot = new Robot(createVector(width / 2, 50))
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
    particles.push(new Particle(random() * width, random() * height, radians(random() * 360)))
  }
  
  //particles.push(new Particle(robot.pos.x, robot.pos.y, robot.heading(), 0.001))
  

  //frameRate(10)
  //noLoop()


}

let vel = 50
let delta_t = 0.1
let yaw_rate
// x, y, theta
let std_pos = [0.25, 0.25, 0.01]
const std_landmark = [50, 50]


function draw() {

  background(0)

  yaw_rate = 0.5//randomGaussian(0, 2)
  
  robot.show()
  
  for(wall of walls) {
    wall.show()
  }
  
  let measurements = robot.check(walls)

  //temp melhorar para filter
  const weights = []

  for (particle of particles) {
    particle.show(measurements)
    particle.predict(delta_t, std_pos, vel, yaw_rate)
    particle.check(walls)

    weights.push(particle.updateWeights(measurements, std_landmark))

  }

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

  robot.move(vel, delta_t, yaw_rate)

  //debug
  let stddevX = Math.sqrt(sigmaX * (1 / N))
  let stddevY = Math.sqrt(sigmaY * (1 / N))

  push()
  fill(0, 200, 0, 50)
  ellipse(meanX, meanY, Math.log(stddevX) * 8, Math.log(stddevY) * 8)
  pop()
}