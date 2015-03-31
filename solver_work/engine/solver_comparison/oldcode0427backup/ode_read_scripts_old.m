function cur_fm = ode_read_scripts_old(filename, STEP)
if STEP < 10
    frame = strcat('mbs_frame00000', num2str(STEP));
else if STEP < 100
        frame = strcat('mbs_frame0000', num2str(STEP));
    else if STEP < 1000
            frame = strcat('mbs_frame000', num2str(STEP));
        else if STEP < 10000
                frame = strcat('mbs_frame00', num2str(STEP));
            else if STEP < 100000
                    frame = strcat('mbs_frame0', num2str(STEP));
                else if STEP < 1000000
                        frame = stracat('mbs_frame', num2str(STEP));
                    end
                end
            end
        end
    end
end
forces = hdf5read(filename, strcat('/', frame, '/bodies/forces'));
inertia_tensors = hdf5read(filename, strcat('/', frame, '/bodies/inertia_tensors'));
masses = hdf5read(filename, strcat('/', frame, '/bodies/masses'));
positions = hdf5read(filename, strcat('/', frame, '/bodies/positions'));
quaternions = hdf5read(filename, strcat('/', frame, '/bodies/quaternions'));                    
velocities = hdf5read(filename, strcat('/', frame, '/bodies/velocities'));

cur_fm.bodies.forces = forces;
cur_fm.bodies.inertia_tensors = inertia_tensors;
cur_fm.bodies.masses = masses;
cur_fm.bodies.positions = positions;
cur_fm.bodies.quaternions = quaternions;
cur_fm.bodies.velocities  = velocities;

gap = hdf5read(filename, strcat('/', frame, '/contacts/gap'));
mu_k = hdf5read(filename, strcat('/', frame, '/contacts/mu_k'));
mu_s = hdf5read(filename, strcat('/', frame, '/contacts/mu_s'));
normals = hdf5read(filename, strcat('/', frame, '/contacts/normals'));
pairs = hdf5read(filename, strcat('/', frame, '/contacts/pairs'));
points = hdf5read(filename, strcat('/', frame, '/contacts/points'));

cur_fm.contacts.gap = gap;
cur_fm.contacts.mu_k = mu_k;
cur_fm.contacts.mu_s = mu_s;
cur_fm.contacts.normals = normals;
cur_fm.contacts.pairs = pairs;
cur_fm.contacts.points = points;


Jacobian = hdf5read(filename, strcat('/', frame, '/constraints/Jacobian'));
constraint_pairs = hdf5read(filename, strcat('/', frame, '/constraints/constraint_pairs'));
lower_bound = hdf5read(filename, strcat('/', frame, '/constraints/lower_bound'));
lower_bound_f = hdf5read(filename, strcat('/', frame, '/constraints/lower_bound_f'));
nconstraints = hdf5read(filename, strcat('/', frame, '/constraints/nconstraints'));
rows = hdf5read(filename, strcat('/', frame, '/constraints/rows'));
time_derivative = hdf5read(filename, strcat('/', frame, '/constraints/time_derivative'));
upper_bound = hdf5read(filename, strcat('/', frame, '/constraints/upper_bound'));
upper_bound_f = hdf5read(filename, strcat('/', frame, '/constraints/upper_bound_f'));
violation = hdf5read(filename, strcat('/', frame, '/constraints/violation'));

cur_fm.constraints.Jacobian = Jacobian;
cur_fm.constraints.constraint_pairs = constraint_pairs;
cur_fm.constraints.lower_bound = lower_bound;
cur_fm.constraints.lower_bound_f = lower_bound_f;
cur_fm.constraints.nconstraints = nconstraints;
cur_fm.constraints.rows = rows;
cur_fm.constraints.time_derivative = time_derivative;
cur_fm.constraints.upper_bound = upper_bound;
cur_fm.constraints.upper_bound_f = upper_bound_f;
cur_fm.constraints.violation = violation;

step_size = hdf5read(filename, strcat('/', frame, '/step_size'));
cur_fm.settings.h = step_size;

end