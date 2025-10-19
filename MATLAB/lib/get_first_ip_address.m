function ip = get_first_ip_address()
   [~, result] = system('hostname');
   hostname = strtrim(result);
   address = java.net.InetAddress.getByName(hostname);
   ip = char(address.getHostAddress());
end