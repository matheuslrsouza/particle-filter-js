
class Particle {

    constructor(x, y, angle) {
        this.measurements = []
        this.pos = createVector(x, y)
        this.dir = p5.Vector.fromAngle(angle)
        this.dir.normalize()
    }

    show() {
        push()
        translate(this.pos.x, this.pos.y)
        fill(0, 255, 0)

        let triangleSize = 15
        rotate(this.dir.heading())
        triangle(0, triangleSize / 4, 0, -triangleSize / 4, triangleSize, 0)
        
        pop()
    }

    predict(delta_t, std_pos, vel, yaw_rate) {
        let theta = this.dir.heading()  
        if (yaw_rate > 0.0001 || yaw_rate < -0.0001) {
            this.pos.x = this.pos.x + (vel / yaw_rate) * (sin(theta + yaw_rate * delta_t) - sin(theta));
            this.pos.y = this.pos.y + (vel / yaw_rate) * (cos(theta) - cos(theta + yaw_rate * delta_t));
            this.dir = p5.Vector.fromAngle(theta + yaw_rate * delta_t);
        } else {
            this.pos.x = this.pos.x + vel * delta_t * cos(theta)
            this.pos.y = this.pos.y + vel * delta_t * sin(theta)
        }

        this.pos.x = randomGaussian(this.pos.x, std_pos[0])
        this.pos.y = randomGaussian(this.pos.y, std_pos[1])
        this.dir = p5.Vector.fromAngle(randomGaussian(this.dir.heading(), std_pos[0]))

    }

    updateWeights(robotMeasurements, std_landmark) {

        let i = 0
        let final_weight = 1.0;        

        // para cada observação do robo, verificamos:
        for (let rMeasurement of robotMeasurements) {
            let bestParticleFit = undefined
            let minDist = Infinity
            
            // 1) qual ponto medido pela particula está mais perto do observado
            // nearest neighbor
            for (let pMeasurement of this.measurements) {
                const d = dist(rMeasurement.point.x, rMeasurement.point.y, pMeasurement.point.x, pMeasurement.point.y)
                if (d < minDist) {
                    minDist = d
                    bestParticleFit = pMeasurement.point
                }
            }

            //debug
            /*let c = [50, 100, 150, 255]

            if (bestParticleFit) {
                push()
                fill(c[i++])
                circle(bestParticleFit.x, bestParticleFit.y, 10)
                circle(rMeasurement.point.x, rMeasurement.point.y, 10)
                pop()
            }*/

            // 2) calcula o peso dado uma distribuição gaussiana multi variavel
            // quanto mais distante o ponto estiver do medido pelo robo, maior a 
            // propabilidade de ser excluída
            if (bestParticleFit) {

                // applying Multivariate_normal_distribution
                // calculate normalization term
                const gauss_norm = (1.0/(2.0 * PI * std_landmark[0] * std_landmark[1]));

                // calculate exponent
                const mu_x = bestParticleFit.x;
                const mu_y = bestParticleFit.y;
                const exponent = (Math.pow(rMeasurement.point.x - mu_x, 2))/(2 * Math.pow(std_landmark[0], 2)) + 
                                                (Math.pow(rMeasurement.point.y - mu_y, 2))/(2 * Math.pow(std_landmark[1], 2));

                // calculate weight using normalization terms and exponent
                final_weight *= gauss_norm * Math.exp(-exponent);
                //console.log(gauss_norm, Math.exp(-exponent))
            }
        }

        return final_weight
    }

    check(walls, robotMeasurements) {
        //create rays for the particle using robot measurements
        let rays = []
        for (let m of robotMeasurements) {
            let ray = new Ray(createVector(this.pos.x, 
                                            this.pos.y),
                                            p5.Vector.fromAngle(m.theta + this.dir.heading() ))
            rays.push(ray)
        }

        // using the created rays, checks if there are walls
        for (ray of rays) {

            let closest
            let minDist = Infinity
            for (wall of walls) {
                let point = ray.intersects(wall)
                if (point) {
                    // get the distance of particle and the wall for the current ray
                    const d = dist(this.pos.x, this.pos.y, point.x, point.y)
                    if (d < minDist) {
                        closest = point
                        minDist = d
                    }
                }
            }
            // for each ray only gets the closest wall (avoid overshooting)
            if (closest) {
                //stroke(255, 100)
                //line(this.pos.x, this.pos.y, closest.x, closest.y)
                this.measurements.push({point: closest})
            }
        }
    }

}