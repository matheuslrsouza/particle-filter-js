window['DEBUG'] = false

let vel = 10
let delta_t = 0.1
let yaw_rate
// x, y, theta
let std_pos_robot = [0.005, 0.005, 0.001]
let std_pos_particle = [1, 1, 5]
const std_landmark = [30, 30]

const maxSteerAngle = Math.PI / 2
let walls
let robot
let filter
let nParticles = 500

//a* algorithm
let n_rows = 50
let n_cols = 50
let grid

let start
let goal

let smooth

let pid

let interval = undefined

function setup() {  
  createCanvas(600, 400)
  noLoop()

  const offsetW = 500;
  const offsetH = 100;

  var btnReset = createButton("Reset")
  btnReset.position(offsetW + 20, offsetH + height + 60);
  btnReset.mousePressed(doSetup)

  checkbox = createCheckbox('Debug Version?', DEBUG);
  checkbox.position(offsetW + 100, offsetH + height + 60);
  checkbox.style('color', 'white');
  checkbox.changed(function() {
    DEBUG = this.checked()
    doSetup()
  });

  var info = createDiv('<b>Shift</b> click to tell robot to follow the path </br>')
    .html('<a href="https://github.com/matheuslrsouza/particle-filter-js">source code</a>', true);
  info.style('color', 'white');
  info.position(offsetW + 20, offsetH + height + 10)
  

  doSetup()  
}

function doSetup() {
  background(220)

  clearInterval(interval)
  interval = undefined

  goal = [parseInt(random() * n_rows), parseInt(random() * n_cols)]
  
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
  
  // create a grid and check if there is wall on map
  // complexity -> length of grid * number of walls
  grid = []
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

  filter = new ParticleFilter(nParticles, robot)

  let i = min(parseInt(robot.pos.x / (width / n_rows)), n_rows - 1)
  let j = min(parseInt(robot.pos.y / (height / n_cols)), n_cols - 1)
  start = [i, j]

  let astar = new AStar(grid, start, goal)
  if (astar.findPath()) { // has path
    let path = []
    let step = astar.goal
    while (step.previous) {
      step = step.previous
      grid[step.x][step.y] = 3
      path.push([step.x, step.y])
    }

    smooth = new Smooth(path.reverse(), grid)
    smooth.calculate()

    // twiddle()
    pid = new PID(smooth.newpath, [22.715382451985896, 4.408941742236364, 0.6665248811689148])
  } else { // there is no path
    console.log('there is no path, trying with another goal')
    doSetup()
  }
  redraw()
}

function initRobot() {
  return new Robot(createVector(100, 50), PI / 2)
}

function twiddle() {
  let tol = 0.0000001
  
  let p = [20.669822917749723, 3.57662533911338, 0.6665368008263248]
  let dp = [1.0, 1.0, 1.0]

  let newPid = new PID(smooth.newpath, p)
  
  let n = 500
  let r = initRobot()
  let bestErr = run(r, n, newPid)

  let it = 0
  while (dp.reduce((acm, cur) => acm + cur) > tol) {
    it++

    for (let i = 0; i < p.length; i++) {

      p[i] += dp[i]

      let r = initRobot()
      let newPid = new PID(smooth.newpath, p)
      let err = run(r, n, newPid)
      

      if (err < bestErr) {
        bestErr = err
        dp[i] *= 1.1 // increase 10%
      } else {
        p[i] -= 2 * dp[i] // try on the opposite direction
        r = initRobot()
        newPid = new PID(smooth.newpath, p)
        err = run(r, n, newPid)
        
        if (err < bestErr) {
          bestErr = err
          dp[i] *= 1.1 // increase 10%
        } else {
          p[i] += dp[i] //back to the original value
          dp[i] *= 0.9 // decrease 10%
        }
      }
    }
    
  }

  console.log('best error: ', p, bestErr)
  pid = new PID(smooth.newpath, p)

}

function run(r, n, p) {

  let err = 0.0

  for (let i = 0; i < 2 * n; i++) {

    let steer = p.getSteer([r.pos.x, r.pos.y])
    r.move(vel, undefined, delta_t, radians(steer)  % 2 * PI)

    if (i >= 10) {
      //begin calculate the error
      err += Math.pow(p.cte, 2)
    }
  }

  return err / n
}

function draw() {

  background(0)

  if (window['DEBUG']) {

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
        }/* else if (content == 5) { //touched (temp)
          console.log('>>>>>>>')
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

  for (let i = 0; i < smooth.newpath.length; i++) {
    push()
    fill(204, 255, 204)
    ellipse(smooth.newpath[i][0] * width / n_rows, smooth.newpath[i][1] * height / n_cols, 5, 5)
    pop()
  }

  //draw the walls
  for(wall of walls) {
    wall.show()
  }

  robot.show()
  
  for (particle of filter.particles) {
    particle.show()
  }

  filter.show()

  // show a flag on the goal
  push()
  beginShape()
  strokeWeight(4);
  stroke(255);
  let px = smooth.newpath[smooth.newpath.length - 1][0] * width / n_rows;
  let py = smooth.newpath[smooth.newpath.length - 1][1] * height / n_cols;
  vertex(px, py)
  vertex(px, py - 30)
  vertex(px + 20, py - 25)
  vertex(px, py - 15)
  endShape()
  pop() 

}

function keyPressed() {
  // must be shift to start the robot 
  // and the robot must not be in the middle of the path
  if (keyCode !== SHIFT || interval !== undefined) {
    console.log('skipping')
    return
  }

  let steer = 0.0
  let lastIndexPath = smooth.newpath.length - 1
  let xGoal = smooth.newpath[lastIndexPath][0] * width / n_rows
  let yGoal = smooth.newpath[lastIndexPath][1] * height / n_cols
  let goal = [xGoal, yGoal]
  let i = 0

  interval = setInterval(function() {
    let measurements = robot.check(walls)
    filter.updateWeights(measurements, walls, std_landmark)
    
    robot.move(vel, std_pos_robot, delta_t, steer)
    filter.predict(delta_t, std_pos_particle, vel, steer)
    
    //resample
    filter.resample()    

    let steerDegree = pid.getSteer([filter.particlePosX(), filter.particlePosY()])
    steer = radians(steerDegree)
    
    let dist = Math.sqrt(
      Math.pow(goal[0] - filter.particlePosX(), 2) + Math.pow(goal[1] - filter.particlePosY(), 2))
    if (dist <= 20) {
      //alert('chegou')
      clearInterval(interval)
      interval = undefined
    } else {
      redraw()
    }
  }, 1)
}

