
class Cell {

    constructor(x, y, g, f, previous) {
        this.x = x
        this.y = y
        this.g = g
        this.f = f
        this.previous = previous
    }

    
}

class AStar {

    open = []
    
    constructor(grid, start, goal) {
        this.grid = grid
        this.start = new Cell(start[0], start[1], 0, 0, undefined)
        this.goal = new Cell(goal[0], goal[1], 0, 0, undefined)

        this.cost = width / this.grid.length * 2
        this.cost = 20

        this.closed = []
        for (let i = 0; i < grid.length; i++) {
            let rowdata = []
            for (let j = 0; j < grid[0].length; j++) {
                rowdata.push(0)
            }
            this.closed.push(rowdata)
        }
    }

    findPath() {
        this.open.push(this.start)
        this.closed[this.start.x][this.start.y] = 1

        let delta = [
            [1, 0],
            [0, -1],
            [-1, 0],
            [0, 1]
        ]

        // let count = 0
        while (true) {
            // count += 1

            if (this.open.length == 0) {
                console.log('no solution!')
                break
            }

            this.open.sort((a, b) => {
                // in reverse order
                return (a.f - b.f) * -1
            })

            let next = this.open.pop()
            let nRows = this.grid.length
            let nCols = this.grid[0].length

            if (next.x == this.goal.x && next.y == this.goal.y) {
                console.log('find!!!')
                this.goal = next
                break
            } else {

                // opened
                this.grid[next.x][next.y] = 2

                for (let i = 0; i < delta.length; i++) {
                    let x2 = next.x + delta[i][0]
                    let y2 = next.y + delta[i][1]
                    
                    // is inner the matrix?
                    if (x2 >= 0 && x2 < nRows && y2 >= 0 && y2 < nCols) {
                        // is not closed and not blocked?
                        if (this.closed[x2][y2] == 0 && this.grid[x2][y2] == 0) {
                            let g2 = next.g + this.cost
                            // euclidean distance without sqrt to maximize the number
                            // in adjacents cells
                            let h2 = 
                                Math.pow(x2 - this.goal.x, 2) + Math.pow(y2 - this.goal.y, 2)
                            // let h2 = abs(x2 - this.goal.x) + abs(y2 - this.goal.y)
                            let f2 = g2 + h2

                            let cell = new Cell(x2, y2, g2, f2)
                            this.open.push(cell)
                            this.closed[x2][y2] = 1

                            // touched
                            this.grid[x2][y2] = 4

                            //who open this cell
                            cell.previous = next
                        }
                    }

                }


            }
                
            

        }

    }


}