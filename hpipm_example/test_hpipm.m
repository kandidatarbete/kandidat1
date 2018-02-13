% Example: Matlab Mex test 
%   Minimize fuel consumption of a vehicle, by optimally varying speed on a
%   hilly terrain. Travel time is constrained such that the vehicle retains
%   a given average speed on the route.

clc;
softcstr=false;
load ocpprob;  % problem data
if ~softcstr, load qpharddata;
else, load qpsoftdata; end

% Some box constraints are not needed. The same solution should be obtained
% if the following three lines are commented out.
ixb(3,:)=false; ixb(5,1:N)=false; 
lb(5,1:N)=NaN; ub(5,1:N)=NaN;
lb(3,:)=NaN; ub(3,:)=NaN; 

% Decrease the penalty of the soft constraints to see a difference
% zu(2,N+1)=0.1;

% Test without general constraints
% C=[]; D=[]; lg=[]; ug=[]; ng=0;

% solve the problem several times to get a reasonable average time
Niter=1000;
avgtime=0;
for iter=1:Niter
    tic;
    if ~softcstr
        [x,u,lam,lamf]=hpipm(N, nx, nu, ng, ngf, x0, A, B, b, ...
            Q, Qf, R, S, q, qf, r, lb, ub, C, D, lg, ug, ...
            Cf, lgf, ugf, ixb);
    else
        [x,u,lam,lamf,s,lams]=hpipm(N, nx, nu, ng, ngf, x0, A, B, b, ...
            Q, Qf, R, S, q, qf, r, lb, ub, C, D, lg, ug, ...
            Cf, lgf, ugf, ixb, Zl, Zu, zl, zu, ixs);
    end
    solvertime=toc; avgtime=avgtime+solvertime;

    cost=0;
    for n=1:N
        Rn=R(:,(n-1)*nu+1:n*nu);
        Qn=Q(:,(n-1)*nx+1:n*nx);
        cost = cost + u(:,n)'*Rn*u(:,n)/2 + x(:,n)'*Qn*x(:,n)/2 + ...
            r(:,n)'*u(:,n) + q(:,n)'*x(:,n);
    end
    cost = cost + qf'*x(:,N+1);

    % save result in a structure
    res=struct;
    res.comptime=solvertime;
    res.cost=cost;
    res.v=sqrt(2*x(1,:)'/ocpprob.m*ocpprob.Se);
    res.t=x(2,:)'*ocpprob.St;
    res.Fe=u(1,:)'*ocpprob.Sf;
    res.Fbrk=u(2,:)'*ocpprob.Sf;
    res.z=u(3,:)'*ocpprob.Sz;
    res.lam=lam;
    res.lamf=lamf;

    % display results
    fuelfcn=@(a,v,Fe) sum(a(1)./v + a(2) + a(3)*v.^2 + a(4)*v.^4 + a(5)*Fe + a(6)*Fe.^2)*ocpprob.ds;
    fuel=fuelfcn(ocpprob.icetrn.parfc,res.v(1:N),res.Fe);
    fprintf('%4.0d: cost=%1.5f (fuel=%1.4f kg), T=%1.4f min, comptime=%1.2f ms\n', ...
        iter,res.cost,fuel/1e3,(res.t(end,:)-res.t(1,:))/60,res.comptime(end)*1000);
end
if Niter > 1
    fprintf('Average comp time=%1.2f ms\n',avgtime/Niter*1e3);
end
plotdata;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2017-12.