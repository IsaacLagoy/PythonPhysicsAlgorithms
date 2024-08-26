import glm

class PhysicsHandler:
    """controls the movement of physics bodies"""
    def __init__(self, scene = None, accelerations:list = [glm.vec3(0, -.8, 0)]) -> None:
        """stores scene and accelerations list in project"""
        self.scene = scene
        self.accelerations = accelerations # constant accelerations in the scene
        
    def update(self, delta_time:float) -> None:
        """moves physics bodies and collections"""
        # update physics bodies 
        if self.scene is None: return # when project has no scene
        physics_body_handler = self.scene.physics_body_handler
        # accelerate all physics bodies by external accelerations
        for body in physics_body_handler.physics_bodies:
            for acceleration in self.accelerations: body.velocity += acceleration * delta_time
        # update collections
        self.scene.collection_handler.update(delta_time)
        
    def add_acceleration(self, acceleration:list) -> None:
        """adds a constant acceleration to project physics engine"""
        self.accelerations.append(acceleration)
        
    def calculate_collisions(self, normal:glm.vec3, collider1, collider2, physics_body1, physics_body2, contact_points:list[glm.vec3]) -> None:
        """resolve the collisions between two objects with multiple contact points"""
        # determine whether or not the colliders have physics
        has_physics1, has_physics2 = physics_body1 is not None, physics_body2 is not None
        # get physics data from valid bodies
        if has_physics1: inv_mass1, inv_inertia1 = 1 / physics_body1.mass, glm.inverse(collider1.get_inertia_tensor(physics_body1.mass))
        if has_physics2: inv_mass2, inv_inertia2 = 1 / physics_body2.mass, glm.inverse(collider2.get_inertia_tensor(physics_body2.mass))
        # gets coefficients
        elasticity = max(collider1.elasticity, collider2.elasticity)
        kinetic = 0.8# min(collider1.kinetic_friction, collider2.kinetic_friction)
        static = min(collider1.static_friction, collider2.static_friction)
        num_points = len(contact_points)
        # calculate impulses from contact points
        if has_physics1 and has_physics2:
            for contact_point in contact_points:
                # apply impulse based reduced by total points
                radius1, radius2 = contact_point - collider1.geometric_center, contact_point - collider2.geometric_center
                impulse = self.calculate_impulse2(physics_body1, physics_body2, inv_mass1, inv_mass2, physics_body1.rotational_velocity * physics_body1.axis_of_rotation, physics_body2.rotational_velocity * physics_body2.axis_of_rotation, radius1, radius2, inv_inertia1, inv_inertia2, elasticity, kinetic, static, normal)
                impulse /= num_points
                # apply impulses
                self.apply_impulse(radius1, impulse, inv_inertia1, inv_mass1, physics_body1)
                self.apply_impulse(radius2, -impulse, inv_inertia2, inv_mass2, physics_body2)
        elif has_physics1:
            for contact_point in contact_points:
                radius = contact_point - collider1.geometric_center
                impulse = self.calculate_impluse1(physics_body1, inv_mass1, physics_body1.rotational_velocity * physics_body1.axis_of_rotation, radius, inv_inertia1, elasticity, kinetic, static, normal)
                impulse /= num_points
                # apply impulses
                self.apply_impulse(radius, -impulse, inv_inertia1, inv_mass1, physics_body1)
        else: # only physics body 2
            for contact_point in contact_points:
                radius = contact_point - collider2.geometric_center
                impulse = self.calculate_impluse1(physics_body2, inv_mass2, physics_body2.rotational_velocity * physics_body2.axis_of_rotation, radius, inv_inertia2, elasticity, kinetic, static, normal)
                impulse /= num_points
                # apply impulse
                self.apply_impulse(radius, impulse, inv_inertia2, inv_mass2, physics_body2)
    
    def precalculate_variables(self, physics_body, collider, contact_point) -> tuple[float]:
        """calculates inverse mass, inverse inertia tensor, and radius to contact point"""
        return 1 / physics_body.mass, glm.inverse(collider.get_inertia_tensor(physics_body.mass)), collider.get_radius_to_point(contact_point)
        
    def calculate_impluse1(self, physics_body, inv_mass, omega, radius, inv_inertia, elasticity, kinetic, static, normal) -> glm.vec3:
        """calculates the impulse from a collision including friction from the impulse"""
        # normal impulse
        relative_velocity = physics_body.velocity + glm.cross(omega, radius)
        relative_normal_velocity = glm.dot(relative_velocity, normal)
        # calculate denominator
        denominator = inv_mass + glm.dot(normal, glm.cross(inv_inertia * glm.cross(radius, normal), radius))
        # calculate normal impulse
        normal_impulse_magnitude = -(1 + elasticity) * relative_normal_velocity / denominator
        normal_impulse = normal_impulse_magnitude * normal
        
        # friction impulse
        rel_tan_vel = relative_velocity - glm.dot(relative_velocity, normal) * normal
        if glm.length(rel_tan_vel) < 1e-6: friction_impulse = glm.vec3(0, 0, 0)
        # kinetic friction TODO: add static friction and if statement
        else: friction_impulse = -kinetic * glm.length(normal_impulse) * glm.normalize(rel_tan_vel)
        # return total impulse
        return glm.vec3([round(v, 3) for v in normal_impulse + friction_impulse])
        
    def calculate_impulse2(self, physics_body1, physics_body2, inv_mass1, inv_mass2, omega1, omega2, radius1, radius2, inv_inertia1, inv_inertia2, elasticity, kinetic, static, normal) -> glm.vec3:
        """calculates the impulse from a collision including friction from the impulse"""
        # normal impulse
        relative_velocity = physics_body1.velocity + glm.cross(omega1, radius1) - (physics_body2.velocity + glm.cross(omega2, radius2))
        relative_normal_velocity = glm.dot(relative_velocity, normal)
        # calculate denominator
        term1 = inv_mass1 + inv_mass2
        term2 = glm.dot(normal, glm.cross(inv_inertia1 * glm.cross(radius1, normal), radius1) + glm.cross(inv_inertia2 * glm.cross(radius2, normal), radius2))
        # calculate normal impulse
        normal_impulse = -(1 + elasticity) * relative_normal_velocity / (term1 + term2) * normal
        
        # friction impulse
        rel_tan_vel = relative_velocity - glm.dot(relative_velocity, normal) * normal
        if glm.length(rel_tan_vel) < 1e6: friction_impulse = glm.vec3(0, 0, 0)
        # kinetic friction TODO: add static friction and if statement
        else: friction_impulse = -kinetic * glm.length(normal_impulse) * glm.normalize(rel_tan_vel)
        # return total impulse
        return glm.vec3([round(v, 3) for v in normal_impulse + friction_impulse])
    
    def apply_impulse(self, radius, impulse_signed, inv_inertia, inv_mass, physics_body) -> None:
        # Update linear velocity
        physics_body.velocity += impulse_signed * inv_mass
        # update rotational velcoity
        delta_omega = inv_inertia * glm.cross(radius, impulse_signed)
        angular_velocity_vector = physics_body.rotational_velocity * physics_body.axis_of_rotation + delta_omega
        if glm.length(angular_velocity_vector) < 1e-6: # checking if there is no rotation
            physics_body.rotational_velocity = 0
            physics_body.axis_of_rotation = glm.vec3(1, 0, 0)
            return
        physics_body.rotational_velocity = glm.length(angular_velocity_vector)
        physics_body.axis_of_rotation = glm.normalize(angular_velocity_vector)
        