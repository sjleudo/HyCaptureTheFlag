function [resultMat, NSH] = multiplyFirstElement(num, mat)
    % Check the sign of the parameter 'num'
    signNum = sign(num);
    
    % Find rows with different sign in the first column
    matchingRows = 0;
    matchingRows = mat(:, 1) * signNum < 0;
    
    % Multiply the first element of matching rows by 100
    mat(matchingRows, 1) = mat(matchingRows, 1) * 100;
    
    % Return the modified matrix
    resultMat = mat;

    %Return number of robots in the same half
    NSH = size(mat ,1)- sum(matchingRows);
end