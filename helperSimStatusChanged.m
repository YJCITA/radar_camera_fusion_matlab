function helperSimStatusChanged
plots = find_system(bdroot,'BlockType','MATLABSystem','System','helperBirdsEyePlot'); 
for i = 1:numel(plots)
    h = get_param(plots{i},'UserData');
    update(h.UserData.SimulinkUIToolbar);
end