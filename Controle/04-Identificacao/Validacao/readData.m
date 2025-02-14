% Definir o diretório atual como caminho dos arquivos
folderPath = pwd; % Obtém o diretório onde o script está rodando

% Lista de arquivos CSV com o padrão 'validação_X.csv'
filePattern = fullfile(folderPath, '*.csv');
files = dir(filePattern);

% Verificar se há arquivos encontrados
if isempty(files)
    error('Nenhum arquivo CSV encontrado no diretório do script.');
end

% Inicializar variável para armazenar os dados
numFiles = numel(files);
dataCell = cell(numFiles,1);

% Ler cada arquivo e armazenar os dados em uma célula
for i = 1:numFiles
    filePath = fullfile(files(i).folder, files(i).name);
    dataCell{i} = readmatrix(filePath, 'NumHeaderLines', 1); % Ignorar cabeçalho
end

% Verificar se todos os arquivos têm o mesmo número de linhas
numRows = cellfun(@(x) size(x,1), dataCell);
if ~all(numRows == numRows(1))
    error('Os arquivos têm números diferentes de linhas.');
end

% Empilhar os dados em um array 3D e calcular a média ao longo da terceira dimensão
dataStack = cat(3, dataCell{:});
meanData = mean(dataStack, 3, 'omitnan'); % Média ponto a ponto, ignorando NaNs

% Exibir a média calculada
disp('Média linha a linha de todos os arquivos:');
disp(meanData);

% Salvar a média em um novo arquivo CSV no mesmo diretório
outputFile = fullfile(folderPath, 'media_validacao.csv');
writematrix(meanData, outputFile);
disp(['Média salva em: ', outputFile]);
