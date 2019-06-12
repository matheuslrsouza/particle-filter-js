
class Smooth {

    constructor(path) {
        this.path = path
        this.newpath = []
        for (let i = 0; i < path.length; i++) {
            this.newpath.push([path[i][0], path[i][1]])
        }
    }

    calculate() {

        let tolerance = 0.000001
        let change = tolerance
        let alfa = 0.1
        let beta = 0.4

        let stepLength = 1

        while (change >= tolerance) {
            change = 0

            for (let i = stepLength; i < this.newpath.length - stepLength; i++) {

                let oldX = this.newpath[i][0]
                let oldY = this.newpath[i][1]

                this.newpath[i][0] += alfa * (this.path[i][0] - this.newpath[i][0]) + 
                    beta * (this.newpath[i + stepLength][0] + this.newpath[i - stepLength][0] - 2 * this.newpath[i][0])
                this.newpath[i][1] += alfa * (this.path[i][1] - this.newpath[i][1]) + 
                    beta * (this.newpath[i + stepLength][1] + this.newpath[i - stepLength][1] - 2 * this.newpath[i][1])

                if (i >= 2) {
                    this.newpath[i][0] += 0.5 * beta *
                        (2.0 * this.newpath[i-1][0] - this.newpath[i-2][0] 
                         - this.newpath[i][0])
                    this.newpath[i][1] += 0.5 * beta *
                         (2.0 * this.newpath[i-1][1] - this.newpath[i-2][1]
                          - this.newpath[i][1])
                }

                if (i <= this.path.len - 4) {
                    this.newpath[i][0] += 0.5 * beta *
                        (2.0 * this.newpath[i+2][0] - this.newpath[i+3][0] 
                         - this.newpath[i][0])
                    this.newpath[i][1] += 0.5 * beta *
                         (2.0 * this.newpath[i+2][1] - this.newpath[i+3][1]
                          - this.newpath[i][1])
                }
                change += abs(oldX - this.newpath[i][0]) + abs(oldY - this.newpath[i][1])
            }            

        }
    }

}