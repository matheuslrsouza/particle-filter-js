class Robot {
  
  constructor(pos, angle) {
    this.size = 30
    this.dir = p5.Vector.fromAngle(angle)
    this.dir.normalize()    
    this.pos = pos
    this.measurements = []
    this.createRays()
  }

  createRays() {
    this.rays = []
    for (let i = 0; i <= 360; i+=10) {
      let angle = this.normalizeAngle(this.dir.heading() + radians(i))
      this.rays.push(new Ray(this.pos, p5.Vector.fromAngle(angle)))
    }
  }

  show() {
    push()
    fill(255, 255, 255)
    translate(this.pos.x, this.pos.y)
    circle(0, 0, this.size)

    rotate(this.dir.heading())
    fill(255, 0, 0)
    let arrowSize = int(this.size / 2);
    translate(arrowSize, 0);
    triangle(0, arrowSize / 2, 0, -arrowSize / 2, arrowSize, 0);

    pop()
    
    //draw measurements (rays)
    push()
    for (let m of this.measurements) {      
      stroke(255, 100)
      line(this.pos.x, this.pos.y, m.point.x, m.point.y)
      stroke(0, 200, 0, 220)
      fill(0, 200, 0, 220)
      circle(m.point.x, m.point.y, 7)
    }
    pop()
  }

  move(vel, std_pos, delta_t, yaw_rate) {
    let theta = this.dir.heading()  

    
    if (yaw_rate > maxSteerAngle) {
      yaw_rate = maxSteerAngle
    } else if (yaw_rate < -maxSteerAngle) {
      yaw_rate = -maxSteerAngle
    }
    
    if (yaw_rate > 0.001 || yaw_rate < -0.001) {
      this.pos.x = this.pos.x + (vel / yaw_rate) * (sin(theta + yaw_rate * delta_t) - sin(theta));
      this.pos.y = this.pos.y + (vel / yaw_rate) * (cos(theta) - cos(theta + yaw_rate * delta_t));
      this.dir = p5.Vector.fromAngle(this.normalizeAngle(theta + yaw_rate * delta_t));
    } else {
      this.pos.x = this.pos.x + vel * delta_t * cos(theta)
      this.pos.y = this.pos.y + vel * delta_t * sin(theta)
    }
    
    if (std_pos !== undefined) {
      this.pos.x = randomGaussian(this.pos.x, std_pos[0])
      this.pos.y = randomGaussian(this.pos.y, std_pos[1])
      let new_deegre = degrees(this.dir.heading())
      let new_radian = this.normalizeAngle(radians(randomGaussian(new_deegre, std_pos[3])))
      this.dir = p5.Vector.fromAngle(new_radian)
    }


    this.createRays()
  }

  check(walls) {

    this.measurements = []

    for (let ray of this.rays) {
      let closest
      let minDist = Infinity
      for (wall of walls) {
        let point = ray.intersects(wall)
        if (point) {
          const d = dist(this.pos.x, this.pos.y, point.x, point.y)
          if (d < minDist) {
            closest = point
            minDist = d
          }
        }
      }
      if (closest) {
        let relativeHeading = ray.dir.heading() - robot.heading()
        relativeHeading = this.normalizeAngle(relativeHeading)
        this.measurements.push({r: minDist, theta: relativeHeading, heading: this.dir.heading(), point: closest})
      }
    }

    return this.measurements
  }

  heading() {
    return this.dir.heading()
  }

  // normalize the angle in order to get only positive values
  normalizeAngle(angle) {
    while (angle < 0) {
        angle += 2 * PI
    }
   return angle
}

}