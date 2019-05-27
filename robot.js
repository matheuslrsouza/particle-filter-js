class Robot {

  
  constructor(pos, angle) {
    this.size = 30
    this.dir = p5.Vector.fromAngle(angle)
    this.pos = pos
    this.rays = []
    for (let i = 0; i < 360; i+=60) {
      let rad = radians(i)
      this.rays.push(new Ray(pos, createVector(cos(rad), sin(rad))))
    }
  }

  show() {
    push()    
    translate(this.pos.x, this.pos.y)
    circle(0, 0, this.size)

    rotate(this.dir.heading())
    fill(255, 0, 0)
    let arrowSize = int(this.size / 2);
    translate(arrowSize, 0);
    triangle(0, arrowSize / 2, 0, -arrowSize / 2, arrowSize, 0);

    pop()
    
    for (ray of this.rays) {
      ray.pos.x = this.pos.x
      ray.pos.y = this.pos.y
      ray.show()
    }
  }

  move(vel, delta_t, yaw_rate) {
    let theta = this.dir.heading()  
    if (yaw_rate > 0.0001 || yaw_rate < -0.0001) {
      this.pos.x = this.pos.x + (vel / yaw_rate) * (sin(theta + yaw_rate * delta_t) - sin(theta));
      this.pos.y = this.pos.y + (vel / yaw_rate) * (cos(theta) - cos(theta + yaw_rate * delta_t));
      this.dir = p5.Vector.fromAngle(theta + yaw_rate * delta_t);
    } else {
      this.pos.x = this.pos.x + vel * delta_t * cos(theta)
      this.pos.y = this.pos.y + vel * delta_t * sin(theta)
    }

  }

  check(walls) {

    let measurements = []

    for (ray of this.rays) {
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
        stroke(255, 100)
        line(this.pos.x, this.pos.y, closest.x, closest.y)
        let relativeHeading = ray.dir.heading() - robot.heading()
        measurements.push({r: minDist, theta: relativeHeading, point: closest})
      }
    }

    return measurements
  }

  heading() {
    return this.dir.heading()
  }

}