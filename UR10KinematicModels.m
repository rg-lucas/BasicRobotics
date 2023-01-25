%% Lucas Resende Gomes - Homework Lecture #4
%% UR10KinematicModels

function  [He, xi_final] = UR10KinematicModels(theta)

% Home configuration
% end-effector pose
He0 = eye(4);

% joint locations
n = 7; % number of active joints
qe = [0;0;0];
q7 = [0;0;-.107];
q6 = [-.088;0;-.107];
q5 = q6;
q4 = [0;0;-.107+.384];
q3 = [-.088;0;-.107+.384];
q2 = [-.088;0;-.107+.384+.316];
q1 = q2;
q0 = [-.088;0;-.107+.384+.316+.33 ];


q = [q0,q1,q2,q3,q4,q5,q6,q7,qe];

% Unit vectors
x = [1; 0; 0]; y = [0; 1; 0]; z = [0; 0; 1];

% Twists of the joints
xi1 = [-z; cross(q1,(-z)) ];
xi2 = [-y; cross(q2,(-y)) ];
xi3 = [-z; cross(q3,(-z)) ];
xi4 = [ y; cross(q4,y)    ];
xi5 = [-z; cross(q5,(-z)) ];
xi6 = [ y; cross(q6,(y))  ];
xi7 = [ z; cross(q7,z)    ];

xi = [xi1,xi2,xi3,xi4,xi5,xi6,xi7];

% joint values
theta0 = zeros(7,1);

% FPK (forward position kinematics) of Franka robot
% given joint configuration
He = He0;
delta_theta = theta - theta0;

% we multiply by starting from the last joint towards the first foint

for i = n:-1:1

  xi_hat = twist2homogenousform( xi(:,i) );
  He = expm( delta_theta(i)*xi_hat )*He;

end

He;
% robot's geometry at its new configuration
% end-effector position

qe_prime = He*[qe;1]; q_prime(:,n+2) = qe_prime(1:3);

% position and screw axis of joints
for i = n:-1:1;
Hi = eye(4);
for k = i-1:-1:1
  xi_hat = twist2homogenousform( xi(:,k) );
  Hi = expm( delta_theta(k)*xi_hat )*Hi;
end

q_prime_temp = Hi*[q(:,i+1); 1]; q_prime(:,i+1) = q_prime_temp(1:3);
H_bar = Hpoint2Hline( Hi );
xi_prime(:,i) = H_bar*xi(:,i);
end

% position of robot's base
q_prime(:,1) = q0;

q_final = q_prime; xi_final = xi_prime; He_final = He;


  % Printing Jacobian Matrix at final joint configuration
 'Jacobian Matrix at Final configuration:';
  xi_final;

  He;
end
