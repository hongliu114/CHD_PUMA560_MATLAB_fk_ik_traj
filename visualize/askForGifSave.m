function save_gif = askForGifSave()
    
    choice = '1';
    % while ~ismember(choice, {'1', '2'})
    %     choice = input('请选择 (1/2): ', 's');
    % end
    
    save_gif = strcmp(choice, '1');
end