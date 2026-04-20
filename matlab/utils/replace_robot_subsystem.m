function replace_robot_subsystem()
% REPLACE_ROBOT_SUBSYSTEM
% Replace robot subsystem in pid_control with template subsystem.
% Reconnect:
%   - Inputs by preserving old source lines
%   - Outputs by directly wiring to 6 PS-Simulink Converter blocks (top->bottom)

    clc;

    %% ===== 1. Config =====
    target_model = 'pid_control';
    target_block = 'pid_control/Gluon_6L3_template';  
    % 如果你在 pid_control 里的旧块名字不是这个，就改这里

    source_model = 'gluon_6l3_template';
    source_block = 'gluon_6l3_template/Gluon_6L3_template';

    %% ===== 2. Load models =====
    load_system(target_model);
    load_system(source_model);

    %% ===== 3. Check source / target =====
    if ~bdIsLoaded(target_model)
        error('Target model not loaded.');
    end
    if ~bdIsLoaded(source_model)
        error('Source model not loaded.');
    end

    %% ===== 4. Record old block position =====
    old_pos = get_param(target_block, 'Position');

    %% ===== 5. Record old INPUT connections =====
    ph = get_param(target_block, 'PortHandles');
    in_lines = get_param(ph.Inport, 'Line');

    if ~iscell(in_lines)
        in_lines = num2cell(in_lines);
    end

    in_src_handles = cell(size(in_lines));
    for i = 1:numel(in_lines)
        lh = in_lines{i};
        if lh ~= -1
            in_src_handles{i} = get_param(lh, 'SrcPortHandle');
        else
            in_src_handles{i} = [];
        end
    end

    %% ===== 6. Find all top-level PS-Simulink Converter blocks =====
    all_blocks = find_system(target_model, 'SearchDepth', 1, 'Type', 'Block');

    ps_blocks = {};
    ps_y = [];

    for i = 1:numel(all_blocks)
        blk = all_blocks{i};
        name = get_param(blk, 'Name');

        if contains(name, 'PS-Simulink Converter')
            ps_blocks{end+1} = blk; %#ok<AGROW>
            pos = get_param(blk, 'Position');
            ps_y(end+1) = pos(2); %#ok<AGROW>
        end
    end

    if numel(ps_blocks) ~= 6
        error('Expected 6 PS-Simulink Converter blocks, but found %d.', numel(ps_blocks));
    end

    % Sort converters from top to bottom
    [~, idx] = sort(ps_y, 'ascend');
    ps_blocks = ps_blocks(idx);

    %% ===== 7. Delete old robot block =====
    delete_block(target_block);

    %% ===== 8. Copy new template block =====
    add_block(source_block, target_block, 'Position', old_pos);

    %% ===== 9. Reconnect INPUTS =====
    new_ph = get_param(target_block, 'PortHandles');

    for i = 1:min(numel(new_ph.Inport), numel(in_src_handles))
        if ~isempty(in_src_handles{i})
            try
                add_line(target_model, in_src_handles{i}, new_ph.Inport(i), 'autorouting', 'on');
            catch ME
                warning('Failed to reconnect input %d: %s', i, ME.message);
            end
        end
    end

    %% ===== 10. Reconnect OUTPUTS directly to PS-Simulink Converters =====
    % First, remove any dangling output lines from the new block
    for i = 1:numel(new_ph.Outport)
        try
            lh = get_param(new_ph.Outport(i), 'Line');
            if lh ~= -1
                delete_line(lh);
            end
        catch
        end
    end

    % Also clear converter input lines if any old dangling lines remain
    for i = 1:numel(ps_blocks)
        ps_ph = get_param(ps_blocks{i}, 'PortHandles');
        try
            lh = get_param(ps_ph.Inport, 'Line');
            if lh ~= -1
                delete_line(lh);
            end
        catch
        end
    end

    % Connect output i -> converter i input
    for i = 1:min(numel(new_ph.Outport), numel(ps_blocks))
        ps_ph = get_param(ps_blocks{i}, 'PortHandles');
        try
            add_line(target_model, new_ph.Outport(i), ps_ph.Inport, 'autorouting', 'on');
            fprintf('Connected output %d -> %s\n', i, ps_blocks{i});
        catch ME
            warning('Failed to connect output %d to %s: %s', i, ps_blocks{i}, ME.message);
        end
    end

    %% ===== 11. Save =====
    save_system(target_model);

    fprintf('\nReplacement finished.\n');
    fprintf('Please inspect pid_control once.\n');
end