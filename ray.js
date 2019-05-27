class Ray {

  constructor(pos, dir) {
    this.pos = pos
    this.dir = dir
  }

  show() {
    push()
    translate(this.pos.x, this.pos.y)
    stroke(255);
    line(0, 0, this.dir.x * 10, this.dir.y * 10)
    pop()

  }

  intersects(wall) {
    let x1 = wall.a.x
    let y1 = wall.a.y
    let x2 = wall.b.x
    let y2 = wall.b.y

    let x3 = this.pos.x
    let y3 = this.pos.y
    let x4 = this.pos.x + this.dir.x
    let y4 = this.pos.y + this.dir.y

    let denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if (denominator == 0) {
      return null
    }

    let numerator = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)

    let t = numerator / denominator
    let u = - ((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator

    let p
    if (t > 0 && t < 1 && u > 0) {
      let x = x1 + t * (x2 - x1)
      let y = y1 + t * (y2 - y1)
      p = createVector(x, y)
    }

    return p
  }


}