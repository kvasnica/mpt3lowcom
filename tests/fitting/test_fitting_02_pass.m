function test_fitting_02_pass
% options must be respected

Double_Integrator
probStruct.Tconstraint = 0;
probStruct.P_N = eye(2);
sysStruct.ymax = [10; 5];
sysStruct.ymin = [-10; -5];
model = mpt_import(sysStruct, probStruct);
model.x.terminalPenalty = model.LQRPenalty();
model.x.with('terminalSet');
model.x.terminalSet = model.LQRSet();
ctrl = EMPCController(model, probStruct.N);

Nempc = 65;
Nfit = 17;

% construct the fitting controller with N=2
fit = FittingController(ctrl, 'N', 2);
assert(fit.nr == Nfit);
assert(fit.optimizer.Num == fit.nr);

% by default invariance must be enforced
assert(ClosedLoop(fit, model).toSystem().isInvariant())

end
