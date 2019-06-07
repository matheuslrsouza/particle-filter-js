window['DEBUG'] = true

let walls
let robot
let particles
let nParticles = 2000

//a* algorithm
let n_rows = 100
let n_cols = 100
let grid = []

let start = [1, 4]
let goal

function setup() {
  goal = [parseInt(random() * n_rows), parseInt(random() * n_cols)]
  // goal = [29,50]
  console.log(goal)

  createCanvas(600, 400)
  background(220)

  // robot = new Robot(createVector(random() * width, random() * height), 0)
  robot = new Robot(createVector(100, 50), PI / 2)
  
  walls = []

  for (let data of mapData) {
    walls.push(new Wall(createVector(data.a.x, data.a.y), createVector(data.b.x, data.b.y)))
  }

  //outside borders
  walls.push(new Wall(createVector(0, 0), createVector(0, height)))
  walls.push(new Wall(createVector(0, 0), createVector(width, 0)))
  walls.push(new Wall(createVector(width, 0), createVector(width, height)))
  walls.push(new Wall(createVector(0, height), createVector(width, height)))

  particles = []
  for (let i = 0; i < nParticles; i++) {
    particles.push(new Particle(random() * width, random() * height, randomGaussian(robot.heading(), 0.1)))
  }

  //particles.push(new Particle(robot.pos.x, robot.pos.y + 0, robot.heading()))
  //particles.push(new Particle(robot.pos.x, robot.pos.y + 100, robot.heading()))
  //particles.push(new Particle(robot.pos.x, robot.pos.y + 200, robot.heading()))
  

  // create a grid and check if there is wall on map
  // complexity -> length of grid * number of walls
  for (let row = 0; row < n_rows; row++) {
    let rowdata = []
    for (let col = 0; col < n_cols; col++) {
      rowdata.push(0)
    }    
    grid.push(rowdata)
  }

  // interpolation between points
  for (let data of walls) {
    let x1 = data.a.x
    let x2 = data.b.x
    let y1 = data.a.y
    let y2 = data.b.y

    let dx = x1 - x2
    let dy = y1 - y2

    if (dx != 0) {
      let m = dy / dx
      let b = y1 - (m * x1)
      for (let x = min(x1, x2); x < max(x1, x2); x++) {

        let y = m * x + b
        let i = min(parseInt(x / (width / n_rows)), n_rows - 1)
        let j = min(parseInt(y / (height / n_cols)), n_cols - 1)
        grid[i][j] = 1
      }

    } else { //same x
      let x = x1
      for (let y = min(y1, y2); y < max(y1, y2); y++) {
        let i = min(parseInt(x / (width / n_rows)), n_rows - 1)
        let j = min(parseInt(y / (height / n_cols)), n_cols - 1)
        grid[i][j] = 1
      }
    }

    

  }

  // frameRate(1)
  noLoop()
}

let vel = 20
let delta_t = 0.1
let yaw_rate
// x, y, theta
let std_pos_robot = [0.9, 0.9, 10]
let std_pos_particle = [3, 3, 10]
const std_landmark = [30, 30]


function draw() {

  background(0)

  yaw_rate = 0.09

  //draw the walls
  for(wall of walls) {
    wall.show()
  }
  
  
  let measurements = robot.check(walls)

  //temp melhorar para filter
  const weights = []

  for (particle of particles) {
    particle.check(walls, measurements)
    particle.predict(delta_t, std_pos_particle, vel, yaw_rate)
    particle.show()

    weights.push(particle.updateWeights(measurements, std_landmark))

  }

  robot.show()
  robot.move(vel, std_pos_robot, delta_t, yaw_rate)
  

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

  let i = parseInt(meanX / (width / n_rows))
  let j = parseInt(meanY / (height / n_cols))

  //grid[i][j] = 1

  //draw grid

  for (let i = 0; i < grid.length; i++) {
    for (let j = 0; j < grid[0].length; j++) {
      let content = grid[i][j]
      push()
      if (content == 1) {
        fill(0, 255, 0)
      } else if (content == 2) { //expanded (temp)
        fill(0, 0, 255)
      } else if (content == 3) { //path (temp)
        fill(255, 0, 102)
      }/* else if (content == 4) { //touched (temp)
        fill(255, 51, 0)
      }*/ else {
        noFill()
      }
      stroke(255, 0, 0, 100)
      rect(i * width / n_rows, j * height / n_cols, width / n_rows, height / n_cols)
      pop()
    }
  }
}

function mousePressed() {

  let i = min(parseInt(mouseX / (width / n_rows)), n_rows - 1)
  let j = min(parseInt(mouseY / (height / n_cols)), n_cols - 1)
  console.log(i, j)


  let astar = new AStar(grid, start, goal)
  astar.findPath()

  let path = []
  let step = astar.goal  
  while (step.previous) {
    step = step.previous
    grid[step.x][step.y] = 3
    path.push([step.x, step.y])
  }

  let smooth = new Smooth(path)
  smooth.calculate()
  
  for (let i = 0; i < smooth.newpath.length; i++) {
    push()
    fill(204, 255, 204)
    console.log(smooth.newpath[i][0] * width / n_rows, smooth.newpath[i][1] * height / n_cols)
    ellipse(smooth.newpath[i][0] * width / n_rows, smooth.newpath[i][1] * height / n_cols, 5, 5)
    pop()
  }


  //redraw()
}