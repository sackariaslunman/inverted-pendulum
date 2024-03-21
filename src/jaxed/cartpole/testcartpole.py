import jax
import jax.numpy as jnp

class Cart1PoleSystem:
    def __init__(
            self, 
            cart_mass: float, 
            gravity: float, 
            masses: jnp.ndarray, 
            lengths: jnp.ndarray, 
            centres_of_mass: jnp.ndarray, 
            frictions: jnp.ndarray,
            inertias: jnp.ndarray
            ) -> None:
        self._initialize(
            cart_mass, 
            gravity, 
            masses, 
            lengths, 
            centres_of_mass, 
            frictions,
            inertias,
            1
        )
        
    def _initialize(
            self, 
            cart_mass: float, 
            gravity: float, 
            masses: jnp.ndarray, 
            lengths: jnp.ndarray, 
            centres_of_mass: jnp.ndarray, 
            frictions: jnp.ndarray,
            inertias: jnp.ndarray,
            n_poles: int,
            ) -> None:
        self.cart_mass = cart_mass
        self.gravity = gravity
        assert masses.shape == (n_poles,)
        self.masses = masses
        assert lengths.shape == (n_poles,)
        self.lengths = lengths
        assert centres_of_mass.shape == (n_poles,)
        self.centres_of_mass = centres_of_mass
        assert frictions.shape == (n_poles,)
        self.frictions = frictions
        assert inertias.shape == (n_poles,)
        self.inertias = inertias

    def __call__(self, state: jnp.ndarray, action: jnp.ndarray) -> jnp.ndarray:
        return self.dynamics(state, action)

    def dynamics(self, state: jnp.ndarray, action: jnp.ndarray) -> jnp.ndarray:
        s, ds, theta1, dtheta1 = state
        g = self.gravity
        mc = self.cart_mass
        m1 = self.masses[0]
        l1 = self.lengths[0]
        a1 = self.centres_of_mass[0]
        d1 = self.frictions[0]
        J1 = self.inertias[0]
        dds = action[0]

        ddtheta1 = (-J1*a1*m1*jnp.sin(theta1)*dtheta1**2+J1*m1*dds+J1*mc*dds \
                    -a1**3*m1**2*jnp.sin(theta1)*dtheta1**2+a1**2*g*m1**2*jnp.sin(2*theta1)*0.5 \
                    -a1**2*m1**2*jnp.cos(theta1)*dds+a1**2*m1**2*dds+a1**2*m1*mc*dds-a1*d1*m1*jnp.cos(theta1)*dtheta1) \
                    / (J1+a1**2*m1)
        dstate = jnp.array([ds, dds, dtheta1, ddtheta1])
        return dstate
    
    def observe(self, state: jnp.ndarray, action: jnp.ndarray) -> jnp.ndarray:
        s, ds, theta1, dtheta1 = state
        g = self.gravity
        mc = self.cart_mass
        m1 = self.masses[0]
        l1 = self.lengths[0]
        a1 = self.centres_of_mass[0]
        d1 = self.frictions[0]
        J1 = self.inertias[0]
        dds = action[0]
        
        tau = (a1*g*m1*jnp.sin(theta1)-a1*m1*jnp.cos(theta1)*dds-d1**dtheta1)/(J1+a1**2*m1)
        return jnp.array([s, ds, theta1, dtheta1, tau])
    
class Cart2PoleSystem(Cart1PoleSystem):
    def __init__(
            self, 
            cart_mass: float, 
            gravity: float, 
            masses: jnp.ndarray, 
            lengths: jnp.ndarray, 
            centres_of_mass: jnp.ndarray, 
            frictions: jnp.ndarray,
            inertias: jnp.ndarray
            ) -> None:
        self._initialize(
            cart_mass, 
            gravity, 
            masses, 
            lengths, 
            centres_of_mass, 
            frictions,
            inertias,
            2
        )

    def dynamics(self, state: jnp.ndarray, action: jnp.ndarray) -> jnp.ndarray:
        s, ds, theta1, dtheta1, theta2, dtheta2 = state
        g = self.gravity
        mc = self.cart_mass
        m1 = self.masses[0]
        l1 = self.lengths[0]
        a1 = self.centres_of_mass[0]
        d1 = self.frictions[0]
        J1 = self.inertias[0]
        m2 = self.masses[1]
        l2 = self.lengths[1]
        a2 = self.centres_of_mass[1]
        d2 = self.frictions[1]
        J2 = self.inertias[1]
        dds = action[0]

        ddtheta1 = (J1*a2*g*m2*jnp.sin(theta2)+J1*a2*l1*m2*jnp.sin(theta1-theta2)*dtheta1**2 \
            -J1*a2*m2*jnp.cos(theta2)*dds+J1*d2*dtheta1-J1*d2*dtheta2+a1**2*a2*g*m1*m2*jnp.sin(theta2) \
            +a1**2*a2*l1*m1*m2*jnp.sin(theta1-theta2)*dtheta1**2 \
            -a1**2*a2*m1*m2*jnp.cos(theta2)*dds*a1**2*d2*m1*dtheta1 \
            -a1**2*d2*m1*dtheta2-a1*a2*g*l1*m1*m2*jnp.sin(2*theta1-theta2)*0.5 \
            -a1*a2*g*l1*m1*m2*jnp.sin(theta2)*0.5+a1*a2*l1*m1*m2*jnp.cos(2*theta1-theta2)*dds*0.5 \
            +a1*a2*l1*m1*m2*jnp.cos(theta2)*dds*0.5+a2**2*l1**2*m2**2*jnp.sin(2*theta1-2*theta2)*dtheta2**2*0.5 \
            +a2*d1*l1*m2*jnp.cos(theta1-theta2)*dtheta1+a2*d2*l1*m2*jnp.cos(theta1-theta2)*dtheta1 \
            -a2*d2*l1*m2*jnp.cos(theta1-theta2)*dtheta2-a2*g*l1**2*m2**2*jnp.sin(2*theta1-theta2)*0.5 \
            +a2*g*l1**2*m2**2*jnp.sin(theta2)*0.5+a2*l1**3*m2**2*jnp.sin(theta1-theta2)*dtheta1**2 \
            +a2*l1**2*m2**2*jnp.cos(2*theta1-theta2)*dds*0.5-a2*l1**2*m2**2*jnp.cos(theta2)*dds*0.5 \
            +d2*l1**2*m2*dtheta1-d2*l1**2*m2*dtheta2)/(J1*J2+J1*a2**2*m2+J2*a1**2*m1 \
            +J2*l1**2*m2+a1**2*a2**2*m1*m2-a2**2*l1**2*m2**2*jnp.cos(theta1-theta2)**2 \
            +a2**2*l1**2*m2**2)
        
        ddtheta2 = 0

        dstate = jnp.array([ds, dds, dtheta1, ddtheta1, dtheta2, ddtheta2])
        return dstate
    
    def observe(self, state: jnp.ndarray, action: jnp.ndarray) -> jnp.ndarray:
        s, ds, theta1, dtheta1, theta2, dtheta2 = state
        g = self.gravity
        mc = self.cart_mass
        m1 = self.masses[0]
        l1 = self.lengths[0]
        a1 = self.centres_of_mass[0]
        d1 = self.frictions[0]
        J1 = self.inertias[0]
        m2 = self.masses[1]
        l2 = self.lengths[1]
        a2 = self.centres_of_mass[1]
        d2 = self.frictions[1]
        J2 = self.inertias[1]
        dds = action[0]

        tau = (J2*a1*g*m1*jnp.sin(theta1)-J2*a1*m1*jnp.cos(theta1)*dds \
                    -J2*a2*l1*m2*jnp.sin(theta1)-theta2*dtheta2**2-J2*d1*dtheta1 \
                    -J2*d2*dtheta1+J2*d2*dtheta2+J2*g*l1*m2*jnp.sin(theta1) \
                    -J2*l1*m2*jnp.cos(theta1)*dds+a1*a2**2*g*m1*m2*jnp.sin(theta1) \
                    -a1*a2**2*m1*m2*jnp.cos(theta1)*dds-a2**3*l1*m2**2*jnp.sin(theta1-theta2)*dtheta2**2 \
                    -a2**2*d1*m2*dtheta1-a2**2*d2*m2*dtheta1+a2**2*d2*m2*dtheta2 \
                    +a2**2*g*l1*m2**2*jnp.sin(theta1-2*theta2)*0.5+a2**2*g*l1*m2**2*jnp.sin(theta1)*0.5 \
                    -a2**2*l1**2*m2**2*jnp.sin(2*theta1-2*theta1)*dtheta1**2*0.5 \
                    +a2**2*l1*m2**2*jnp.cos(theta1-2*theta2)*dds*0.5-a2**2*l1*m2**2*jnp.cos(theta1)*dds*0.5 \
                    -a2*d2*l1*m2*jnp.cos(theta1-theta2)*dtheta1+a2*d2*l1*m2*jnp.cos(theta1-theta2)*dtheta2) \
                    /(J1*J2+J2*a2**2*m2+J2*a1**2*m1+J2*l1**2*m2 \
                    +a1**2*a2**2*m1*m2-a2**2*l1**2*m2*2*jnp.cos(theta1-theta2)**2+a2**2*l1**2*m2**2)
        
        dstate = jnp.array([ds, dds, dtheta1, dtheta2, tau])
        return dstate
    
def test_cart1polesystem():
    cart_mass = 1.0
    gravity = 9.81
    masses = jnp.array([1.0])
    lengths = jnp.array([1.0])
    half_lengths = lengths * 0.56
    frictions = jnp.array([1.0])
    inertias = jnp.array([1.0])
    system = Cart1PoleSystem(cart_mass, gravity, masses, lengths, half_lengths, frictions, inertias)
    state = jnp.array([0.0, 0.0, 0.0, 0.0])
    action = jnp.array([0.1])
    print(system(state, action))


if __name__ == "__main__":
    test_cart1polesystem()