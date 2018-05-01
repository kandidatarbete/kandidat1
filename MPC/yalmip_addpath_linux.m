function yalmip_addpath()
    root='YALMIPdevelop201709';
    folders={'extras','demos','solvers','modules','modules/parametric',...
        'modules/moment','modules/global','modules/robust','modules/sos', ...
        'operators'};

    addpath(root);
    for i=1:numel(folders)
        addpath([root,filesep,folders{i}]);
    end

     addpath('ecos');
end