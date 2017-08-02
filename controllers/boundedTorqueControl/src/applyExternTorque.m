function tau_ext = applyExternTorque(t, model)

    %t: simulation time

    %Apply an external torque to the joint
    if t >= model.TextIniTime && t <= model.TextEndTime
        tau_ext = model.Text;
    else
        tau_ext = zeros(length(model.Text), 1);
    end
end