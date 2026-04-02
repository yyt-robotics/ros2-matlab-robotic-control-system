function q = get_joint_ref_http(~)
    url = 'http://127.0.0.1:5002/joint_ref';

    try
        data = webread(url);
        q = reshape(data.joints, 1, 6);   % 输出 1x6
    catch
        q = zeros(1, 6);
    end
end