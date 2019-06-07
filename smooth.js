
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
        let alfa = 0.9
        let beta = 0.5

        let stepLength = 3

        while (change >= tolerance) {
            change = 0

            for (let i = stepLength; i < this.newpath.length - stepLength; i++) {

                let oldX = this.newpath[i][0]
                let oldY = this.newpath[i][1]

                this.newpath[i][0] += alfa * (this.path[i][0] - this.newpath[i][0]) + 
                    beta * (this.newpath[i + stepLength][0] + this.newpath[i - stepLength][0] - 2 * this.newpath[i][0])
                this.newpath[i][1] += alfa * (this.path[i][1] - this.newpath[i][1]) + 
                    beta * (this.newpath[i + stepLength][1] + this.newpath[i - stepLength][1] - 2 * this.newpath[i][1])

                change += abs(oldX - this.newpath[i][0]) + abs(oldY - this.newpath[i][1])
            }            

        }

        console.log(this.newpath)
    }

}