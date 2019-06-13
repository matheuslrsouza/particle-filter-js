
class ParticleFilter {

    constructor(N, robot) {
        this.weights = []
        this.x = 0
        this.y = 0
        this.stddevX = 0
        this.stddevY = 0        
        this.particles = []

        for (let i = 0; i < N; i++) {
            this.particles.push(new Particle(
                randomGaussian(robot.pos.x, 10),
                randomGaussian(robot.pos.y, 10),
                randomGaussian(robot.heading(), 0.3))
            )
        }
    }

    updateWeights(measurements, walls, std_landmark) {
        this.weights = []

        for (particle of this.particles) {
            particle.check(walls, measurements)
            this.weights.push(particle.updateWeights(measurements, std_landmark))
        }
    }

    predict(delta_t, std_pos_particle, vel, steer) {
        for (particle of this.particles) {
            particle.predict(delta_t, std_pos_particle, vel, steer)
        }
    }

    resample() {
        //resample
        const maxW = Math.max.apply(null, this.weights)
        let beta = 0.0
        const N = this.weights.length
        let index = parseInt(random() * N)

        let selectedParticles = []

        let meanX = 0
        let meanY = 0

        let sigmaX = 0
        let sigmaY = 0
        
        for (let i = 0; i < N; i++) {
            beta += random() * 2.0 * maxW
            while (beta > this.weights[index]) {
                beta -= this.weights[index]
                index = (index + 1) % N
            }
            const p = this.particles[index]
            selectedParticles.push(new Particle(p.pos.x, p.pos.y, p.dir.heading()))
            
            // particle position
            meanX += p.pos.x / N
            meanY += p.pos.y / N
            
            let sigmaSquareX = Math.pow(p.pos.x - meanX, 2)
            let sigmaSquareY = Math.pow(p.pos.y - meanY, 2)

            sigmaX += sigmaSquareX
            sigmaY += sigmaSquareY
        }

        this.stddevX = Math.sqrt(sigmaX * (1 / N))
        this.stddevY = Math.sqrt(sigmaY * (1 / N))

        this.x = meanX
        this.y = meanY
        this.particles = selectedParticles
    }

    show() {
        push()
        strokeWeight(5)
        stroke(0, 255, 0, 200)
        noFill()
        // fill(0, 0, 255, 100)
        ellipse(this.particlePosX(), filter.particlePosY(), 
                Math.log(this.stddevX) * 12, Math.log(this.stddevY) * 12)
        pop()
    }

    particlePosX() {
        return this.x
    }

    particlePosY() {
        return this.y
    }

}