import React, {type ReactNode} from 'react';
import Layout from '@theme-original/Layout';
import type LayoutType from '@docusaurus/theme-classic';
import type {WrapperProps} from '@docusaurus/types';
import ChatbotWidget from '../../components/ChatbotWidget'; // Import your ChatbotWidget component

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): ReactNode {
  return (
    <>
      <Layout {...props}>
        {props.children}
      </Layout>
      <ChatbotWidget />
    </>
  );
}
